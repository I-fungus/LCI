#include "LCI_Structs.h"

#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#define NOMINMAX
#include <Windows.h>

namespace
{
struct PlotPoint
{
    double time = 0.0;
    double latitude = 0.0;
    double longitude = 0.0;
};

std::mutex g_plotMutex;
std::vector<PlotPoint> g_points;
std::thread g_plotThread;
HWND g_plotWindow = nullptr;
bool g_plotRunning = false;
double g_originLatitude = 0.0;
double g_originLongitude = 0.0;
bool g_hasOrigin = false;

constexpr wchar_t kPlotWindowClass[] = L"LCIRealtimePlotWindow";

std::vector<POINT> BuildScreenPoints(const std::vector<PlotPoint>& points, const RECT& rect)
{
    std::vector<POINT> screenPoints;
    if (points.empty())
    {
        return screenPoints;
    }

    const int leftMargin = 55;
    const int rightMargin = 25;
    const int topMargin = 30;
    const int bottomMargin = 45;
    const int rawWidth = static_cast<int>(rect.right - rect.left) - leftMargin - rightMargin;
    const int rawHeight = static_cast<int>(rect.bottom - rect.top) - topMargin - bottomMargin;
    const int width = std::max(1, rawWidth);
    const int height = std::max(1, rawHeight);

    std::vector<double> eastMeters;
    std::vector<double> northMeters;
    eastMeters.reserve(points.size());
    northMeters.reserve(points.size());

    const double cosLat = std::cos(g_originLatitude);
    for (const PlotPoint& point : points)
    {
        eastMeters.push_back((point.longitude - g_originLongitude) * R_WGS84 * cosLat);
        northMeters.push_back((point.latitude - g_originLatitude) * R_WGS84);
    }

    const auto [minEastIt, maxEastIt] = std::minmax_element(eastMeters.begin(), eastMeters.end());
    const auto [minNorthIt, maxNorthIt] = std::minmax_element(northMeters.begin(), northMeters.end());
    const double minEast = *minEastIt;
    const double maxEast = *maxEastIt;
    const double minNorth = *minNorthIt;
    const double maxNorth = *maxNorthIt;

    const double eastCenter = 0.5 * (minEast + maxEast);
    const double northCenter = 0.5 * (minNorth + maxNorth);
    const double eastRange = std::max(1.0, maxEast - minEast);
    const double northRange = std::max(1.0, maxNorth - minNorth);
    const double scale = 0.92 * std::min(
        static_cast<double>(width) / eastRange,
        static_cast<double>(height) / northRange);
    const double xCenter = rect.left + leftMargin + 0.5 * width;
    const double yCenter = rect.top + topMargin + 0.5 * height;

    screenPoints.reserve(points.size());
    for (std::size_t i = 0; i < points.size(); ++i)
    {
        POINT point;
        point.x = static_cast<LONG>(std::lround(xCenter + (eastMeters[i] - eastCenter) * scale));
        point.y = static_cast<LONG>(std::lround(yCenter - (northMeters[i] - northCenter) * scale));
        screenPoints.push_back(point);
    }

    return screenPoints;
}

void DrawPlot(HWND hwnd, HDC hdc)
{
    RECT rect;
    GetClientRect(hwnd, &rect);

    HBRUSH background = CreateSolidBrush(RGB(250, 250, 250));
    FillRect(hdc, &rect, background);
    DeleteObject(background);

    std::vector<PlotPoint> points;
    {
        std::lock_guard<std::mutex> lock(g_plotMutex);
        points = g_points;
    }

    SetBkMode(hdc, TRANSPARENT);
    SetTextColor(hdc, RGB(30, 30, 30));

    const std::wstring title = L"Real-time GNSS/INS Loose Coupling Trajectory";
    TextOutW(hdc, 16, 10, title.c_str(), static_cast<int>(title.size()));

    if (points.empty())
    {
        const std::wstring waiting = L"Waiting for realtime navigation states...";
        TextOutW(hdc, 16, 45, waiting.c_str(), static_cast<int>(waiting.size()));
        return;
    }

    const int leftMargin = 55;
    const int rightMargin = 25;
    const int topMargin = 30;
    const int bottomMargin = 45;
    RECT axisRect{
        rect.left + leftMargin,
        rect.top + topMargin,
        rect.right - rightMargin,
        rect.bottom - bottomMargin
    };

    HPEN axisPen = CreatePen(PS_SOLID, 1, RGB(160, 160, 160));
    HPEN oldPen = static_cast<HPEN>(SelectObject(hdc, axisPen));
    Rectangle(hdc, axisRect.left, axisRect.top, axisRect.right, axisRect.bottom);
    SelectObject(hdc, oldPen);
    DeleteObject(axisPen);

    std::vector<POINT> screenPoints = BuildScreenPoints(points, rect);
    if (screenPoints.size() >= 2)
    {
        HPEN routePen = CreatePen(PS_SOLID, 2, RGB(0, 105, 180));
        oldPen = static_cast<HPEN>(SelectObject(hdc, routePen));
        Polyline(hdc, screenPoints.data(), static_cast<int>(screenPoints.size()));
        SelectObject(hdc, oldPen);
        DeleteObject(routePen);
    }

    const POINT& current = screenPoints.back();
    HBRUSH currentBrush = CreateSolidBrush(RGB(210, 40, 40));
    HBRUSH oldBrush = static_cast<HBRUSH>(SelectObject(hdc, currentBrush));
    Ellipse(hdc, current.x - 4, current.y - 4, current.x + 4, current.y + 4);
    SelectObject(hdc, oldBrush);
    DeleteObject(currentBrush);

    const PlotPoint& last = points.back();
    wchar_t status[256];
    swprintf_s(
        status,
        L"Points: %zu    Time: %.3f s    Lat: %.10f deg    Lon: %.10f deg",
        points.size(),
        last.time,
        last.latitude * RadToDeg,
        last.longitude * RadToDeg);
    TextOutW(hdc, 16, rect.bottom - 28, status, static_cast<int>(wcslen(status)));

    const std::wstring xLabel = L"East";
    const std::wstring yLabel = L"North";
    TextOutW(hdc, axisRect.right - 35, axisRect.bottom + 6, xLabel.c_str(), static_cast<int>(xLabel.size()));
    TextOutW(hdc, 8, axisRect.top + 8, yLabel.c_str(), static_cast<int>(yLabel.size()));
}

LRESULT CALLBACK PlotWindowProc(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    switch (message)
    {
    case WM_PAINT:
    {
        PAINTSTRUCT ps;
        HDC hdc = BeginPaint(hwnd, &ps);
        DrawPlot(hwnd, hdc);
        EndPaint(hwnd, &ps);
        return 0;
    }
    case WM_DESTROY:
        g_plotWindow = nullptr;
        PostQuitMessage(0);
        return 0;
    default:
        return DefWindowProcW(hwnd, message, wParam, lParam);
    }
}

void PlotThreadMain()
{
    HINSTANCE instance = GetModuleHandleW(nullptr);
    WNDCLASSW windowClass{};
    windowClass.lpfnWndProc = PlotWindowProc;
    windowClass.hInstance = instance;
    windowClass.lpszClassName = kPlotWindowClass;
    windowClass.hCursor = LoadCursor(nullptr, IDC_ARROW);
    windowClass.hbrBackground = reinterpret_cast<HBRUSH>(COLOR_WINDOW + 1);
    RegisterClassW(&windowClass);

    g_plotWindow = CreateWindowExW(
        0,
        kPlotWindowClass,
        L"LCI Real-time Trajectory",
        WS_OVERLAPPEDWINDOW,
        CW_USEDEFAULT,
        CW_USEDEFAULT,
        900,
        650,
        nullptr,
        nullptr,
        instance,
        nullptr);

    if (g_plotWindow == nullptr)
    {
        return;
    }

    ShowWindow(g_plotWindow, SW_SHOW);
    UpdateWindow(g_plotWindow);

    MSG message;
    while (GetMessageW(&message, nullptr, 0, 0) > 0)
    {
        TranslateMessage(&message);
        DispatchMessageW(&message);
    }
}
}

void StartRealTimePlot()
{
    if (g_plotRunning)
    {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(g_plotMutex);
        g_points.clear();
        g_hasOrigin = false;
    }

    g_plotRunning = true;
    g_plotThread = std::thread(PlotThreadMain);
}

void AddRealTimePlotPoint(double time, double latitude, double longitude)
{
    if (!g_plotRunning)
    {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(g_plotMutex);
        if (!g_hasOrigin)
        {
            g_originLatitude = latitude;
            g_originLongitude = longitude;
            g_hasOrigin = true;
        }
        g_points.push_back(PlotPoint{time, latitude, longitude});
    }

    HWND hwnd = g_plotWindow;
    if (hwnd != nullptr)
    {
        InvalidateRect(hwnd, nullptr, FALSE);
    }
}

void StopRealTimePlot()
{
    if (!g_plotRunning)
    {
        return;
    }

    HWND hwnd = g_plotWindow;
    if (hwnd != nullptr)
    {
        PostMessageW(hwnd, WM_CLOSE, 0, 0);
    }

    if (g_plotThread.joinable())
    {
        g_plotThread.join();
    }

    g_plotRunning = false;
}
