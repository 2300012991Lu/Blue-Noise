#include "Labs/Final/tasks.h"
#include "Engine/app.h"
#include "Engine/loader.h"

#include <numbers>
#include <vector>
#include <iostream>
#include <fstream>
#include <random>

using VCX::Labs::Common::ImageRGB;
namespace VCX::Labs::Final {

    template <typename FloatType>
    class UniformDistribution {
        std::random_device rd;
        std::default_random_engine eng;
        std::uniform_real_distribution<FloatType> dist;
    public:
        UniformDistribution(FloatType minVal, FloatType maxVal): rd(), eng(rd()), dist(minVal, maxVal) {  }
        FloatType random() { return dist(eng); }
        template <typename IntType>
        IntType randint(IntType maxVal) { return std::min((IntType)(dist(eng) * maxVal), maxVal - 1); }
    };

    static UniformDistribution<float> uniform(0.f, 1.f);

    template <typename dtype>
    class NBox {
        size_t Length;
        std::vector<dtype> data;
        UniformDistribution<float>& dist = uniform;
        size_t GetIndex() const {
            return dist.randint(Length);
        }
    public:

        NBox(const std::vector<dtype>& _init_Data = {}) : data(_init_Data) {
            Length = data.size();
        }
        NBox(const NBox& _box) = delete;
        NBox(NBox&& __box) = delete;
        NBox& operator = (const NBox& _box) = delete;

        const std::vector<dtype>& All() const { return data; }
        size_t size() const { return Length; }

        void Push(const dtype& _elem) {
            data.push_back(_elem);
            Length++;
        }
        dtype& Get() {
            if (Length == 0) {
                std::cerr << "Cannot get an element from an empty Box" << std::endl;
                throw 0;
            }
            return data[GetIndex()];
        }
        void Erase(dtype& _elem) {
            if (Length == 0) {
                std::cerr << "Cannot erase an element from an empty Box" << std::endl;
                throw 0;
            }
            std::swap(_elem, data[Length - 1]);
            data.pop_back();
            --Length;
        }

        bool Empty() const {
            return Length == 0;
        }
    };


    void FinalTask(Common::ImageRGB& canvas, const double r, const size_t sampleRate, const size_t iterTimes, const std::string& file) {
        
        using Point = std::pair<double, double>;
        using Box = NBox<Point>;
        static constexpr size_t K = 30;     // For each 
        static constexpr size_t size = 960; // The size of canvas is 960 * 960

        static constexpr int neighbours[21][2] = {
                     {-2,-1}, {-2, 0}, {-2, 1},
            {-1,-2}, {-1,-1}, {-1, 0}, {-1, 1}, {-1, 2},
            { 0,-2}, { 0,-1},          { 0, 1}, { 0, 2},
            { 1,-2}, { 1,-1}, { 1, 0}, { 1, 1}, { 1, 2},
                     { 2,-1}, { 2, 0}, { 2, 1}
        };

        static bool globalOccupy[size][size];
        static size_t array[size][size];    // Record times any pixel was sampled during iteration
        static float color[size][size];     // Record average sampled times of pixels in range of sampleRate * sampleRate

        static double oldRadius = 0;
        static size_t oldIterTimes = 0;
        static size_t oldSampleRate = 0;

        static std::string oldFile = "badDemand";
        static float bright = 0;

        bool changedCompletely = false;

        if (oldIterTimes != iterTimes || oldRadius != r) {

            double d = r / std::sqrt(2);
            size_t nsize = (size_t)((double)size / d) + 1;
            bool** occupy = new bool* [nsize];
            Point** occupy_position = new Point* [nsize];
            for (size_t i = 0; i != nsize; ++i) {
                occupy[i] = new bool[nsize];
                occupy_position[i] = new Point[nsize];
            }
            changedCompletely = true;           // Sample changed

            oldRadius = r;
            memset(color, 0, sizeof(color));    // In IEEE-754 , float 0b00...00 is 0

            size_t iterStart = 0;
            if (oldIterTimes < iterTimes) iterStart = oldIterTimes;
            else memset(array, 0, sizeof(array));
            oldIterTimes = iterTimes;

            for (size_t _ = iterStart; _ != iterTimes; ++_) { // iterate to get more layor of noise
                memset(globalOccupy, 0, sizeof(globalOccupy));
                for (size_t i = 0; i!= nsize; ++i) memset(occupy[i], 0, sizeof(bool) * nsize);

                std::vector<Point> sampleList;

                size_t X0 = uniform.randint(size), Y0 = uniform.randint(size);
                size_t idxX = (size_t)((double)X0 / d), idxY = (size_t)((double)Y0 / d);
                Point newPoint((double)X0, (double)Y0);
                occupy[idxX][idxY] = true;
                occupy_position[idxX][idxY] = newPoint;
                Box active_list({newPoint});
                sampleList.push_back(newPoint);

                while (!active_list.Empty()) {
                    Point& point = active_list.Get();
                    size_t iter = 0;
                    for (; iter != K; ++iter) {
                        // Generate Normal Distribution On Ring
                        /*
                            p(theta) = 1 / 2pi    theta in (0,2pi)
                            p(x) = (2 / 3) * x        x in (1,2)
                            So we just need to consider how to make a distribution for x
                            We can try to build a distribution in which p(x) ~ linear(x)
                            Then move axis to fit q(x) = (2/3) * x
                        */
                        double theta = uniform.random() * 3.1415926 * 2, x = uniform.random() + uniform.random();
                        x = x < 1 ? x + 1 : uniform.random() + 1;
                        // You can proof that p(x) = (2/3) * x for x in (1,2)
                        double newX = point.first + x * glm::cos(theta) * r;
                        double newY = point.second + x * glm::sin(theta) * r;
                        if (std::round(newX) < 0 || std::round(newY) < 0 || std::round(newX) >= (double)size || std::round(newY) >= (double)size) continue;
                        idxX = (int)(newX / d);
                        idxY = (int)(newY / d);
                        if (occupy[idxX][idxY]) continue;
                        bool checkFail = false;
                        double r2 = r * r;
                        for (size_t j = 0; j != 20; ++j) {
                            size_t checkX = idxX + neighbours[j][0], checkY = idxY + neighbours[j][1];
                            if (checkX >= nsize || checkY >= nsize) continue; // unsigned integer which < 0 will overflow
                            if (occupy[checkX][checkY]) {
                                Point& p = occupy_position[checkX][checkY];
                                if ((newX - p.first) * (newX - p.first) + (newY - p.second) * (newY - p.second) < r2) {
                                    checkFail = true;
                                    break;
                                }
                            }
                        }
                        if (checkFail) continue;
                        newPoint = Point(newX, newY);
                        occupy[idxX][idxY] = true;
                        occupy_position[idxX][idxY] = newPoint;
                        active_list.Push(newPoint);
                        sampleList.push_back(newPoint);
                        break;
                    }
                    if (iter == K) active_list.Erase(point); // reject
                }

                for (auto& npoint : sampleList) {
                    size_t _X = (size_t)std::round(npoint.first);
                    size_t _Y = (size_t)std::round(npoint.second);
                    if (!globalOccupy[_X][_Y]) {
                        globalOccupy[_X][_Y] = true;
                        ++array[_X][_Y];
                    }
                }
            }
            delete [] occupy;
            delete [] occupy_position;
        }

        if (changedCompletely || oldSampleRate != sampleRate) { // Sample changed, we should re-paint the canvas
            oldSampleRate = sampleRate;
            float totalBright = 0;

            for (size_t i = 0; i != size / sampleRate; ++i) {
                for (size_t j = 0; j != size / sampleRate; ++j) {
                    size_t baseX = i * sampleRate;
                    size_t baseY = j * sampleRate;
                    size_t count = 0;
                    for (size_t u = baseX; u != baseX + sampleRate; ++u) {
                        for (size_t v = baseY; v != baseY + sampleRate; ++v) {
                            count += array[u][v];
                        }
                    }
                    totalBright += sampleRate * sampleRate * iterTimes - count;
                    float colorUV = 1 - (float)count / (float)(sampleRate * sampleRate * iterTimes);
                    for (size_t u = baseX; u != baseX + sampleRate; ++u) {
                        for (size_t v = baseY; v != baseY + sampleRate; ++v) {
                            color[u][v] = colorUV;
                        }
                    }
                }
            }

            float total = (size / sampleRate) * (size / sampleRate) * sampleRate * sampleRate * iterTimes;
            bright = totalBright / total;
        }

        if (oldFile != file) {
            oldFile = file;

            if (file != "") {
                Common::ImageRGBA _input = Engine::LoadImageRGBA(file);
                size_t sizeX = _input.GetSizeX(), sizeY = _input.GetSizeY();
                Common::ImageRGB input = Common::AlphaBlend(
                    _input,
                    Common::CreatePureImageRGB(sizeX, sizeY, { 0, 0, 0 }));
                size_t bX = 0, bY = 0, lX = size, lY = size;
                size_t bU = 0, bV = 0;
                size_t zoom = std::min(size / sizeX, size / sizeY);
                if (sizeX < size) {
                    lX = sizeX;
                    bX = (size - sizeX * zoom) >> 1;
                    bU = (size - sizeX) >> 1;
                }
                if (sizeY < size) {
                    lY = sizeY;
                    bY = (size - sizeY * zoom) >> 1;
                    bV = (size - sizeY) >> 1;
                }
                if (zoom == 0) zoom = 1;
                for (size_t u = 0; u != lX; ++u) {
                    for (size_t v = 0; v != lY; ++v) {
                        glm::vec3 _color = input.At(u, v);
                        float __color = (_color.r + _color.g + _color.b) / 3;
                        glm::vec3 colorUV{ (color[bU + u][bV + v] / (2 * bright) + __color) > 1.f ? 1.f : 0.f };
                        size_t iX = bX + u * zoom, iY = bY + v * zoom;
                        size_t jX = iX + zoom, jY = iY + zoom;
                        for (size_t x = iX; x != jX; ++x) {
                            for (size_t y = iY; y != jY; ++y) {
                                canvas.At(x, y) = colorUV;
                            }
                        }
                    }
                }
                return;
            }
        }

        for (size_t i = 0; i != size; ++i) {
            for (size_t j = 0; j != size; ++j) {
                float _color = color[i][j];
                canvas.At(i, j) = glm::vec3{ _color, _color, _color };
            }
        }

        return;
    }


    class VectorN { // We will use VectorN as the index of n-dimensional array
        size_t dim;
        std::vector<double> position;

        static void rec(int depth, int bound, const VectorN& lastVector, std::vector<VectorN>& vecs) {
            if (depth < 0) {
                vecs.push_back(lastVector);
                return;
            }
            for (int i = -bound; i <= bound; ++i) {
                VectorN v = lastVector;
                v[depth] = (double)i;
                rec(depth - 1, bound, v, vecs);
            }
        }
    public:

        static std::vector<VectorN> GetNeighbours(const size_t _dim) {
            int bound = (int)std::ceil(std::sqrt(_dim));
            std::vector<VectorN> ret;
            std::vector<double> primary(_dim, 0);
            rec((int)_dim - 1, bound, VectorN(primary), ret);
            return ret;
        }

        VectorN(size_t _dim): dim(_dim) , position(_dim) {  }
        VectorN(const std::vector<double>& _position): dim(_position.size()), position(_position) {  }
        double& operator [] (const size_t where) { return position[where]; }
        const double& operator [] (const size_t where) const { return position[where]; }

        void operator = (const VectorN& vec) {
            dim = vec.dim;
            position.clear();
            position.reserve(dim);
            for (int i = 0; i < dim; ++i) {
                position.push_back(vec.position[i]);
            }
        }
        
        VectorN operator + (const VectorN& vec) const {
            if (dim != vec.dim) throw 2;
            std::vector<double> ret(dim);
            for (size_t i = 0; i != dim; ++i) {
                ret[i] = position[i] + vec.position[i];
            }
            return VectorN(ret);
        }
        VectorN operator - (const VectorN& vec) const {
            if (dim != vec.dim) throw 2;
            std::vector<double> ret(dim);
            for (size_t i = 0; i != dim; ++i) {
                ret[i] = position[i] - vec.position[i];
            }
            return VectorN(ret);
        }
        VectorN operator * (const double k) const {
            std::vector<double> ret(dim);
            for (size_t i = 0; i != dim; ++i) {
                ret[i] = position[i] * k;
            }
            return VectorN(ret);
        }
        VectorN operator / (const double k) const {
            std::vector<double> ret(dim);
            for (size_t i = 0; i != dim; ++i) {
                ret[i] = std::floor(position[i] / k);
            }
            return VectorN(ret);
        }
        
        double Distance(const VectorN& vec) const {
            if (dim != vec.dim) throw 2;
            double D2 = 0;
            for (size_t i = 0; i != dim; ++i) {
                double delta = (position[i] - vec.position[i]);
                D2 += delta * delta;
            }
            return std::sqrt(D2);
        }
        double Length() const {
            double D2 = 0;
            for (size_t i = 0; i != dim; ++i) {
                D2 += position[i] * position[i];
            }
            return std::sqrt(D2);
        }

        bool InBoundary(int minval, int maxval) const {
            for (size_t i = 0; i != dim; ++i) {
                if (std::round(position[i]) < minval) return false;
                if (std::round(position[i]) >= maxval) return false;
            }
            return true;
        }

        int ToInt(int dimLength) const {
            int ret = 0;
            for (size_t i = 0; i != dim; ++i) {
                ret = ret * dimLength + (int)position[i];
            }
            return ret;
        }
        void FromInt(int index, int dimLength) {
            for (int i = (int)dim - 1; i >= 0; --i) {
                position[i] = double(index % dimLength);
                index /= dimLength;
            }
        }

        size_t GetDim() const { return dim; }
        const std::vector<double>& GetPosition() const { return position; }
    };

    template <typename T>
    class ArrayN { // n-dimensional array
        size_t dim;
        size_t dimLength;
        std::vector<T> data;
    public:
        friend class VectorN;
        ArrayN(size_t _dim, size_t _dimLength) : dim(_dim), dimLength(_dimLength), data((size_t)std::pow(_dimLength, _dim)) {  }
        ArrayN(size_t _dim, size_t _dimLength, const T& InitElement) : dim(_dim), dimLength(_dimLength), data((size_t)std::pow(_dimLength, _dim), InitElement) {  }

        T& At(const VectorN& pos) {
            if (pos.GetDim() != dim) throw 3;
            return data[pos.ToInt(dimLength)];
        }
    };


    void NDimension(const double r, const size_t n, const size_t l) {

        double d = r / std::sqrt(n);
        auto neighbours = VectorN::GetNeighbours(n);

        ArrayN<int> occupy(n, (size_t)std::ceil(l / d), false);
        ArrayN<VectorN> occupyPosition(n, (size_t)std::ceil(l / d), VectorN(n));

        NBox<VectorN> ActiveList;
        std::vector<VectorN> SampleList;

        std::vector<double> _P0(n);
        for (int i = 0; i < n; ++i) {
            _P0[i] = (double)uniform.randint(l);
        }
        VectorN P0{_P0};
        VectorN idxP0 = P0 / d;

        occupy.At(idxP0) = true;
        occupyPosition.At(idxP0) = P0;
        ActiveList.Push(P0);
        SampleList.push_back(P0);

        while (!ActiveList.Empty()) {
            VectorN& point = ActiveList.Get();
            size_t iter = 0;
            for (; iter != 30; ++iter) {
                VectorN Annulus(n);
                while (true) {
                    std::vector<double> _Annulus(n);
                    for (int i = 0; i < n; ++i) {
                        _Annulus[i] = uniform.random() * 4 - 2;
                    }
                    Annulus = VectorN(_Annulus);
                    double length = Annulus.Length();
                    if (length > 1 && length < 2) break;
                }
                VectorN newPoint = point + Annulus * r;
                VectorN idxNewPoint = newPoint / d;
                if (!idxNewPoint.InBoundary(0, (size_t)std::ceil(l / d))) continue;
                if (occupy.At(idxNewPoint)) continue;
                bool checkFail = false;
                for (auto neighborDist : neighbours) {
                    VectorN checkP = idxNewPoint + neighborDist;
                    if (!checkP.InBoundary(0, (size_t)std::ceil(l / d))) continue;
                    if (occupy.At(checkP)) {
                        VectorN& p = occupyPosition.At(checkP);
                        if (newPoint.Distance(p) < r) {
                            checkFail = true;
                            break;
                        }
                    }
                }
                if (checkFail) continue;
                occupy.At(idxNewPoint) = true;
                occupyPosition.At(idxNewPoint) = newPoint;
                ActiveList.Push(newPoint);
                SampleList.push_back(newPoint);
                break;
            }
            if (iter == 30) ActiveList.Erase(point);
        }

        std::string filename = "Noise R=" + std::to_string(r).substr(0, 5) + " , n=" + std::to_string(n) + " , l=" + std::to_string(l) + ".txt";
        std::ofstream fout(filename);

        for (const auto& point : SampleList) {
            const auto& data = point.GetPosition();
            for (const auto& pos : data) {
                fout << pos << ' ';
            }
            fout << std::endl;
        }

        fout.close();

        return;
    }

}; // namespace VCX::Labs::Final