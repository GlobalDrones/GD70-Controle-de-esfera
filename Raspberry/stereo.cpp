#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>

#include <iostream>
#include <thread>
#include <mutex>
#include <atomic>
#include <deque>
#include <chrono>
#include <cmath>
#include <vector>

#ifdef __linux__
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#endif

using namespace std;
using namespace cv;

// ===========================================================================
// CONFIGURACOES SERIAL
// ===========================================================================

static const int BAUDRATE = 115200;

#ifdef __linux__
class ArduinoSerial {
public:
    int fd = -1;

    bool connect() {
        vector<string> portas = {
            "/dev/ttyUSB0",
            "/dev/ttyUSB1",
            "/dev/ttyUSB2",
            "/dev/ttyACM0",
            "/dev/ttyACM1"
        };

        cout << "[INFO] Procurando Arduino nas portas USB..." << endl;

        for (const auto& porta : portas) {
            fd = open(porta.c_str(), O_RDWR | O_NOCTTY | O_SYNC);

            if (fd < 0)
                continue;

            struct termios tty{};
            if (tcgetattr(fd, &tty) != 0) {
                close(fd);
                fd = -1;
                continue;
            }

            cfsetospeed(&tty, B115200);
            cfsetispeed(&tty, B115200);

            tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
            tty.c_iflag &= ~IGNBRK;
            tty.c_lflag = 0;
            tty.c_oflag = 0;
            tty.c_cc[VMIN] = 0;
            tty.c_cc[VTIME] = 1;

            tty.c_iflag &= ~(IXON | IXOFF | IXANY);
            tty.c_cflag |= (CLOCAL | CREAD);
            tty.c_cflag &= ~(PARENB | PARODD);
            tty.c_cflag &= ~CSTOPB;
            tty.c_cflag &= ~CRTSCTS;

            tcsetattr(fd, TCSANOW, &tty);

            cout << "[OK] Conectado ao Arduino: " << porta << endl;
            return true;
        }

        cout << "[AVISO] Arduino nao encontrado." << endl;
        return false;
    }

    void sendAngle(float angle) {
        if (fd < 0)
            return;

        angle = max(0.0f, min(90.0f, angle));

        string msg = to_string(angle) + "\n";
        write(fd, msg.c_str(), msg.size());
    }

    void closePort() {
        if (fd >= 0)
            close(fd);
    }
};
#endif

// ===========================================================================
// CONFIG CAMERAS
// ===========================================================================

static const string CAM0_ID = "rtsp://192.168.144.25:8554/main.264";
static const string CAM1_ID = "rtsp://192.168.144.2:8554/main.264";

static const int FRAME_W = 1280;
static const int FRAME_H = 720;

// ===========================================================================
// ASYNC CAMERA
// ===========================================================================

class AsyncCamera {
public:
    AsyncCamera(const string& src, const string& name, int w, int h)
        : src(src), name(name), width(w), height(h) {}

    bool start() {
        cout << "[INFO] Conectando " << name << "..." << endl;

        cap.open(src);

        if (!cap.isOpened()) {
            cerr << "[ERRO] Falha ao abrir: " << src << endl;
            return false;
        }

        cap.set(CAP_PROP_FRAME_WIDTH, width);
        cap.set(CAP_PROP_FRAME_HEIGHT, height);
        cap.set(CAP_PROP_BUFFERSIZE, 1);

        for (int i = 0; i < 5; i++)
            cap.read(dummy);

        running = true;

        th = thread(&AsyncCamera::loop, this);

        cout << "[OK] " << name << " conectada" << endl;

        return true;
    }

    Mat read() {
        lock_guard<mutex> lk(mtx);

        if (frame.empty())
            return Mat();

        return frame.clone();
    }

    void stop() {
        running = false;

        if (th.joinable())
            th.join();

        cap.release();
    }

private:
    void loop() {
        while (running) {
            Mat f;

            if (cap.read(f) && !f.empty()) {
                lock_guard<mutex> lk(mtx);
                frame = f;
            } else {
                this_thread::sleep_for(chrono::milliseconds(5));
            }
        }
    }

    string src;
    string name;

    int width;
    int height;

    VideoCapture cap;

    Mat frame;
    Mat dummy;

    mutex mtx;

    atomic<bool> running{false};

    thread th;
};

// ===========================================================================
// PARAMETROS
// ===========================================================================

void loadParams(
    Mat& cmtx0,
    Mat& dist0,
    Mat& cmtx1,
    Mat& dist1,
    Mat& R_rel,
    Mat& T_rel
) {
    cmtx0 = (Mat_<double>(3,3) <<
        849.09475804, 0, 656.00357451,
        0, 827.44435633, 407.89408896,
        0, 0, 1);

    dist0 = (Mat_<double>(1,5) <<
        0.29794075, -0.2598218, 0, 0, 0);

    cmtx1 = (Mat_<double>(3,3) <<
        810.15131758, 0, 669.59465007,
        0, 783.52171627, 382.90946542,
        0, 0, 1);

    dist1 = (Mat_<double>(1,5) <<
        0.28443683, -0.27500358, 0, 0, 0);

    R_rel = (Mat_<double>(3,3) <<
        0.99989951, 0.0131155, 0.00538065,
        -0.01301987, 0.99976317, -0.01743832,
        -0.00560808, 0.01736651, 0.99983346);

    T_rel = (Mat_<double>(3,1) <<
        -0.13558734,
        0.00144692,
        0.01341408);
}

// ===========================================================================
// DETECCAO DA LINHA MAIS PROXIMA
// ===========================================================================

bool detectarLinhaMaisProxima(
    const Mat& rect_l,
    const Mat& disp,
    float focal,
    float baseline,
    int cx,
    int cy,
    int roi_radius,
    float& out_angle,
    Vec4i& best_line,
    float& best_dist,
    Mat& roi_bin_out
) {
    Mat gray;
    cvtColor(rect_l, gray, COLOR_BGR2GRAY);

    Mat blurred;
    GaussianBlur(gray, blurred, Size(5,5), 0);

    Mat bin_img;
    threshold(blurred, bin_img, 60, 255, THRESH_BINARY_INV);

    Mat mask = Mat::zeros(gray.size(), CV_8U);

    circle(mask, Point(cx, cy), roi_radius, Scalar(255), -1);

    Mat roi_bin;
    bitwise_and(bin_img, bin_img, roi_bin, mask);

    roi_bin_out = roi_bin.clone();

    Mat edges;
    Canny(roi_bin, edges, 50, 150);

    vector<Vec4i> lines;

    HoughLinesP(
        edges,
        lines,
        1,
        CV_PI / 180,
        40,
        roi_radius / 3,
        20
    );

    if (lines.empty())
        return false;

    float menor_dist = 1e9f;

    Vec4i melhor;

    for (const auto& l : lines) {
        int x1 = l[0];
        int y1 = l[1];
        int x2 = l[2];
        int y2 = l[3];

        int num_points = (int)hypot(x2 - x1, y2 - y1);

        if (num_points <= 0)
            continue;

        vector<float> profundidades;

        for (int i = 0; i < num_points; i++) {
            float t = (float)i / (float)num_points;

            int px = (int)(x1 + t * (x2 - x1));
            int py = (int)(y1 + t * (y2 - y1));

            if (px < 0 || py < 0 ||
                px >= disp.cols || py >= disp.rows)
                continue;

            float d = disp.at<float>(py, px);

            if (d > 0.0f) {
                float Z = (focal * baseline) / d;

                if (Z <= 3.0f)
                    profundidades.push_back(Z);
            }
        }

        if (profundidades.size() > num_points * 0.3f) {
            float media = 0.0f;

            for (float z : profundidades)
                media += z;

            media /= profundidades.size();

            if (media < menor_dist) {
                menor_dist = media;
                melhor = l;
            }
        }
    }

    if (menor_dist == 1e9f)
        return false;

    float angle = atan2(
        (float)(melhor[2] - melhor[0]),
        (float)(melhor[3] - melhor[1])
    ) * 180.0f / CV_PI;

    if (angle < 0)
        angle = 90.0f + abs(angle);

    out_angle = angle;
    best_line = melhor;
    best_dist = menor_dist;

    return true;
}

// ===========================================================================
// MAIN
// ===========================================================================

int main() {

#ifdef __linux__
    ArduinoSerial arduino;
    arduino.connect();
#endif

    cv::setNumThreads(4);

    Mat cmtx0, dist0, cmtx1, dist1, R_rel, T_rel;

    loadParams(
        cmtx0,
        dist0,
        cmtx1,
        dist1,
        R_rel,
        T_rel
    );

    Size imgSize(FRAME_W, FRAME_H);

    Mat R1, R2, P1, P2, Q;

    stereoRectify(
        cmtx0,
        dist0,
        cmtx1,
        dist1,
        imgSize,
        R_rel,
        T_rel,
        R1,
        R2,
        P1,
        P2,
        Q,
        0
    );

    Mat map1x, map1y, map2x, map2y;

    initUndistortRectifyMap(
        cmtx0,
        dist0,
        R1,
        P1,
        imgSize,
        CV_32FC1,
        map1x,
        map1y
    );

    initUndistortRectifyMap(
        cmtx1,
        dist1,
        R2,
        P2,
        imgSize,
        CV_32FC1,
        map2x,
        map2y
    );

    float focal = (float)P1.at<double>(0,0);
    float baseline = abs((float)T_rel.at<double>(0,0));

    AsyncCamera cam0(CAM0_ID, "cam0-esq", FRAME_W, FRAME_H);
    AsyncCamera cam1(CAM1_ID, "cam1-dir", FRAME_W, FRAME_H);

    if (!cam0.start() || !cam1.start())
        return -1;

    Ptr<StereoSGBM> left_matcher = StereoSGBM::create(
        0,
        64,
        5
    );

    left_matcher->setP1(8 * 3 * 5 * 5);
    left_matcher->setP2(32 * 3 * 5 * 5);
    left_matcher->setMode(StereoSGBM::MODE_SGBM_3WAY);

    Ptr<cv::StereoMatcher> right_matcher =
        cv::ximgproc::createRightMatcher(left_matcher);

    Ptr<cv::ximgproc::DisparityWLSFilter> wls =
        cv::ximgproc::createDisparityWLSFilter(left_matcher);

    wls->setLambda(8000);
    wls->setSigmaColor(1.5);

    namedWindow("Stereo Profundidade", WINDOW_NORMAL);

    deque<float> historico(1);

    double fps = 0.0;

    auto last = chrono::high_resolution_clock::now();

    while (true) {

        Mat f0 = cam0.read();
        Mat f1 = cam1.read();

        if (f0.empty() || f1.empty()) {
            this_thread::sleep_for(chrono::milliseconds(5));
            continue;
        }

        Mat rect_l, rect_r;

        remap(f0, rect_l, map1x, map1y, INTER_LINEAR);
        remap(f1, rect_r, map2x, map2y, INTER_LINEAR);

        Mat gl, gr;

        cvtColor(rect_l, gl, COLOR_BGR2GRAY);
        cvtColor(rect_r, gr, COLOR_BGR2GRAY);

        Mat dl, dr;

        left_matcher->compute(gl, gr, dl);
        right_matcher->compute(gr, gl, dr);

        Mat disp;

        wls->filter(dl, rect_l, disp, dr);

        disp.convertTo(disp, CV_32F, 1.0 / 16.0);

        Mat mask = disp > 0;

        Mat disp_norm;

        normalize(disp, disp_norm, 0, 255, NORM_MINMAX);

        disp_norm.convertTo(disp_norm, CV_8U);

        Mat disp_vis;

        applyColorMap(disp_norm, disp_vis, COLORMAP_JET);

        disp_vis.setTo(Scalar(0,0,0), ~mask);

        int cx = rect_l.cols / 2;
        int cy = rect_l.rows / 2;

        int roi_radius = 80;

        float angulo;
        Vec4i linha;
        float dist_alvo;

        Mat roi_bin;

        bool found = detectarLinhaMaisProxima(
            rect_l,
            disp,
            focal,
            baseline,
            cx,
            cy,
            roi_radius,
            angulo,
            linha,
            dist_alvo,
            roi_bin
        );

        Mat hough_vis = rect_l.clone();

        Mat bin_vis;

        cvtColor(roi_bin, bin_vis, COLOR_GRAY2BGR);

        if (found) {

            if (angulo > 90)
                angulo = 180 - angulo;

            historico.push_back(angulo);

            if (historico.size() > 1)
                historico.pop_front();

            float media = 0;

            for (float a : historico)
                media += a;

            media /= historico.size();

#ifdef __linux__
            arduino.sendAngle(media);
#endif

            line(
                disp_vis,
                Point(linha[0], linha[1]),
                Point(linha[2], linha[3]),
                Scalar(0,255,0),
                3
            );

            line(
                hough_vis,
                Point(linha[0], linha[1]),
                Point(linha[2], linha[3]),
                Scalar(0,255,0),
                3
            );

            line(
                bin_vis,
                Point(linha[0], linha[1]),
                Point(linha[2], linha[3]),
                Scalar(0,255,0),
                3
            );

            string txt =
                "Dist: " + to_string(dist_alvo) +
                "m | Ang: " + to_string(media);

            putText(
                hough_vis,
                txt,
                Point(10, 30),
                FONT_HERSHEY_SIMPLEX,
                0.7,
                Scalar(0,255,0),
                2
            );
        }

        circle(disp_vis, Point(cx, cy), roi_radius, Scalar(255,255,255), 1);
        circle(hough_vis, Point(cx, cy), roi_radius, Scalar(0,255,255), 2);
        circle(bin_vis, Point(cx, cy), roi_radius, Scalar(0,255,255), 1);

        putText(
            hough_vis,
            "Cam Esquerda",
            Point(10, rect_l.rows - 20),
            FONT_HERSHEY_SIMPLEX,
            0.6,
            Scalar(0,255,255),
            2
        );

        putText(
            bin_vis,
            "Binarizacao",
            Point(10, rect_l.rows - 20),
            FONT_HERSHEY_SIMPLEX,
            0.6,
            Scalar(255,255,255),
            2
        );

        putText(
            disp_vis,
            "Profundidade",
            Point(10, rect_l.rows - 20),
            FONT_HERSHEY_SIMPLEX,
            0.6,
            Scalar(255,255,255),
            2
        );

        Mat display;

        hconcat(vector<Mat>{
            hough_vis,
            bin_vis,
            disp_vis
        }, display);

        auto now = chrono::high_resolution_clock::now();

        double dt =
            chrono::duration<double>(now - last).count();

        fps = 0.9 * fps + 0.1 / max(dt, 1e-6);

        last = now;

        putText(
            display,
            "FPS: " + to_string((int)fps),
            Point(10, 25),
            FONT_HERSHEY_SIMPLEX,
            0.7,
            Scalar(0,255,255),
            2
        );

        imshow("Stereo Profundidade", display);

        int k = waitKey(1) & 0xFF;

        if (k == 'q' || k == 27)
            break;
    }

    cam0.stop();
    cam1.stop();

#ifdef __linux__
    arduino.closePort();
#endif

    destroyAllWindows();

    return 0;
}