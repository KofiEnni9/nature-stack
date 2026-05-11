#include "nature/thirdparty/glm/glm.hpp"
#include <vector>
#include <Eigen/Dense>

using mat3x3 = glm::mat3x3;
// using mat3x6 = glm::mat<3,6,float>;
// using mat6x6 = glm::mat<6,6,float>;

struct vec6 {
    float data[6] = {};
    float& operator[](int i) { return data[i]; }
    float operator[](int i) const { return data[i]; }

    vec6& operator+=(const vec6& o) {
        for(int i = 0;i<6;++i) data[i] +=o.data[i]; return *this;
    }
    vec6& operator+=(float s) {
        for(int i = 0;i<6;++i) data[i] +=s; return *this;
    }
    vec6& operator/=(float s) {
        for(int i = 0;i<6;++i) data[i] /=s; return *this;
    }
    vec6 operator-(float s) const {
        vec6 r; for(int i = 0;i<6;++i) r.data[i] = data[i]-s; return r;
    }
    vec6& operator*=(float s) {
        for(int i = 0;i<6;++i) data[i] *=s; return *this;
    }

};

struct mat3x6
{
    vec6 data[3];
    vec6& operator[](int i) { return data[i]; }
    const vec6& operator[](int i) const { return data[i]; }

    mat3x6& operator+=(float o) {
        for(int i = 0;i<3;++i)
            data[i] += o;
        return *this;
    }
    mat3x6& operator+=(mat3x6 o) {
        for(int i = 0;i<3;++i)
            data[i] += o.data[i];
        return *this;
    }
    mat3x6& operator/=(float s) {
        for(int i = 0;i<3;++i)
            data[i] /= s;
        return *this;
    }
};

struct mat6x6
{
    vec6 data[6];
    vec6& operator[](int i) { return data[i]; }
    const vec6& operator[](int i) const { return data[i]; }

    mat6x6& operator+=(float o) {
        for(int i = 0;i<6;++i)
            data[i] += o;
        return *this;
    }
    mat6x6& operator+=(mat6x6 o) {
        for(int i = 0;i<6;++i)
            data[i] += o.data[i];
        return *this;
    }
    mat6x6& operator/=(float s) {
        for(int i = 0;i<6;++i)
            data[i] /= s;
        return *this;
    }
};

mat3x6 outerProduct(const glm::vec3& a, const vec6& b) {
    mat3x6 m;
    for(int row=0; row<3; row++)
    for(int col=0; col<6; col++)
        
            m[row][col] = a[row] * b[col];

    return m;
}

mat6x6 outerProduct(const vec6& a, const vec6& b) {
    mat6x6 m;
    for(int col=0; col<6; col++)
        for(int row=0; row<6; row++)
            m[col][row] = a[row] * b[col];

    return m;
}

glm::vec3 operator*(const mat3x6& m, const vec6& v) {
    glm::vec3 result(0.0f);
    for (int row = 0; row < 3; ++row)
        for (int col = 0; col < 6; ++col)
            result[row] += m[row][col] * v[col];
    return result;
}

vec6 operator*(const mat6x6& m, const vec6& v) {
    vec6 result;
    for (int row = 0; row < 6; ++row)
        for (int col = 0; col < 6; ++col)
            result[row] += m[row][col] * v[col];
    return result;
}

float Dot(const vec6& a, const vec6& b) {
    float s = 0;
    for (int i = 0; i < 6; ++i) s += a[i] * b[i];
    return s;
}

float Dot(const glm::vec3& a, const glm::vec3& b) {
    return glm::dot(a, b);
}






void Preprocess (int n, glm::vec3 points[], glm::vec3 X[], glm::vec3 &average, vec6& mu, mat3x3 &F0, mat3x6& F1, mat6x6 &F2)
{
    average = {0, 0, 0};
    for(int i = 0; i < n;++i)
    {
        average += points[i];
    }
    average /= n;
    for(int i = 0; i<n; ++i)
    {
        X[i] = points[i] - average;
    }
    vec6 zero = {0, 0, 0, 0, 0, 0};
    std::vector<vec6> products(n);
    mu = zero;
    for(int i = 0; i< n; ++i)
    {
        products[i][0] = X[i][0] * X[i][0];
        products[i][1] = X[i][0] * X[i][1];
        products[i][2] = X[i][0] * X[i][2];
        products[i][3] = X[i][1] * X[i][1];
        products[i][4] = X[i][1] * X[i][2];
        products[i][5] = X[i][2] * X[i][2];
        mu[0] += products[i][0];
        mu[1] += 2 * products[i][1];
        mu[2] += 2 * products[i][2];
        mu[3] += products[i][3];
        mu[4] += 2 * products[i][4];
        mu[5] += products[i][5];
    }
    mu /= n;

    F0 = mat3x3(0);
    // F1 = mat3x6(0);
    // F2 = mat6x6(0);
    for(int i = 0; i < n; ++i)
    {
        vec6 delta;
        delta[0] = products[i][0] - mu[0];
        delta[1] = 2 * products[i][1] - mu[1];
        delta[2] = 2 * products[i][2] - mu[2];
        delta[3] = products[i][3] - mu[3];
        delta[4] = 2 * products[i][4] - mu[4];
        delta[5] = products[i][5] - mu[5];
        F0[0][0] += products[i][0];
        F0[0][1] += products[i][1];
        F0[0][2] += products[i][2];
        F0[1][1] += products[i][3];
        F0[1][2] += products[i][4];
        F0[2][2] += products[i][5];
        F1 += outerProduct(X[i], delta);
        F2 += outerProduct(delta, delta);
    }
    F0 /= n;
    F0[1][0] = F0[0][1];
    F0[2][0] = F0[0][2];
    F0[2][1] = F0[1][2];
    F1 /= n;
    F2 /= n;

}

float Trace(const mat3x3 m) {
    return m[0][0] + m[1][1] + m[2][2];
}

float G(int n, glm::vec3 X[], vec6& mu, mat3x3 &F0, mat3x6& F1, mat6x6 &F2, glm::vec3 W, glm::vec3& PC, float& rSqr)
{
    mat3x3 P = mat3x3(1.0f) - glm::outerProduct(W,W);
    mat3x3 S(0, -W[2], W[1], W[2], 0, -W[0], -W[1], W[0], 0);
    mat3x3 A = P * F0 * P;
    mat3x3 hatA = -(S * A * S);
    mat3x3 hatAA = hatA * A;
    float trace = Trace(hatAA);
    mat3x3 Q = hatA / trace;
    vec6 p = { P[0][0], P[0][1], P[0][2], P[1][1], P[1][2], P[2][2] };
    glm::vec3 alpha = F1 * p;
    glm::vec3 beta = Q * alpha;
    float error = (Dot(p , F2 * p) - 4 * Dot(alpha, beta) + 4*Dot(beta, F0 * beta)) / n;
    PC = beta;
    rSqr = Dot(p, mu) + Dot(beta, beta);
    return error;
}


float FitCylinder (int n, glm::vec3 points[], float& rSqr, glm::vec3& C, glm::vec3& W)
{
    glm::vec3 X[n];
    glm::vec3 average;
    vec6 mu;
    mat3x3 F0;
    mat3x6 F1;
    mat6x6 F2;
    Preprocess(n, points, X, average, mu, F0, F1, F2);

    const float halfPi = M_PI * 0.5f;
    const float twoPi = M_PI * 2.0f;
    const int jmax = 64;
    const int imax = 64;

    W = glm::vec3(0.0f, 0.0f, 1.0f);
    glm::vec3 currentC;
    float currentRSqr;
    float minError = G(n, X, mu, F0, F1, F2, W, currentC, currentRSqr);
    C = currentC + average;
    rSqr = currentRSqr;

    // float minError = INFINITY;
    // W = glm::vec3(0, 0, 0);
    // C = glm::vec3(0, 0, 0);
    // rSqr = 0;
    // for(int j = 0; j <= jmax; ++j)
    // {
    //     float phi = halfPi * j/jmax;
    //     float csphi = cos(phi), snphi = sin(phi);
    //     for (int i = 0; i < imax; ++i)
    //     {
    //         float theta = twoPi * i/imax;
    //         float cstheta = cos(theta), sntheta = sin(theta);
    //         glm::vec3 currentW(cstheta * snphi, sntheta * snphi, csphi); 
    //         glm::vec3 currentC;
    //         float currentRSqr;
    //         float error = G(n, X, mu, F0, F1, F2, currentW, currentC, currentRSqr);
    //         if(error < minError)
    //         {
    //             minError = error;
    //             W = currentW;
    //             C = currentC;
    //             rSqr = currentRSqr;
    //         }
    //     }
    // }

    // C += average;

    return minError;

}
