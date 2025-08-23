#ifndef PROJECTILE_ANGLE_CONVEL_H
#define PROJECTILE_ANGLE_CONVEL_H

#include <iostream>
#include <complex>
#include <algorithm>
#include <cmath>

#define MUZZLE_VELOCITY 25000
#define SHOT_IMPOSSIBLE -100
#define PI 3.14159
#define RAD_2_DEG 180/PI

#define OoM -10

/**
 * @author sidneycadot
 * @source: https://github.com/sidneycadot/quartic 
 * @brief Below three are helper functions from the above source to solve the quartic arising from the motion
 *        intersection equations
 */

static std::complex<double> complex_sqrt(const std::complex<double> & z)
{
    return pow(z, 1. / 2.);
}

static std::complex<double> complex_cbrt(const std::complex<double> & z)
{
    return pow(z, 1. / 3.);
}

void solve_quartic(const std::complex<double> coefficients[5], std::complex<double> roots[4])
{
    // The algorithm below was derived by solving the quartic in Mathematica, and simplifying the resulting expression by hand.

    const std::complex<double> a = coefficients[0];
    const std::complex<double> b = coefficients[1] / a;
    const std::complex<double> c = coefficients[2] / a;
    const std::complex<double> d = coefficients[3] / a;
    const std::complex<double> e = coefficients[4] / a;

    const std::complex<double> Q1 = c * c - 3. * b * d + 12. * e;
    const std::complex<double> Q2 = 2. * c * c * c - 9. * b * c * d + 27. * d * d + 27. * b * b * e - 72. * c * e;
    const std::complex<double> Q3 = 8. * b * c - 16. * d - 2. * b * b * b;
    const std::complex<double> Q4 = 3. * b * b - 8. * c;

    const std::complex<double> Q5 = complex_cbrt(Q2 / 2. + complex_sqrt(Q2 * Q2 / 4. - Q1 * Q1 * Q1));
    const std::complex<double> Q6 = (Q1 / Q5 + Q5) / 3.;
    const std::complex<double> Q7 = 2. * complex_sqrt(Q4 / 12. + Q6);

    roots[0] = (-b - Q7 - complex_sqrt(4. * Q4 / 6. - 4. * Q6 - Q3 / Q7)) / 4.;
    roots[1] = (-b - Q7 + complex_sqrt(4. * Q4 / 6. - 4. * Q6 - Q3 / Q7)) / 4.;
    roots[2] = (-b + Q7 - complex_sqrt(4. * Q4 / 6. - 4. * Q6 + Q3 / Q7)) / 4.;
    roots[3] = (-b + Q7 + complex_sqrt(4. * Q4 / 6. - 4. * Q6 + Q3 / Q7)) / 4.;
}



/**
 * @brief Solves for the pitch yaw angles using gravity model
 *        Includes vector helper functions
 * 
 */

struct vec3{
    double x = 0, y = 0, z = 0;
    std::string id = "N/A";

    vec3(){x=0;y=0;z=0;}

    vec3(double a, double b, double c)
    {
        x = a;
        y = b;
        z = c;
    }

    vec3 operator + (vec3 const &a)
    {
        vec3 b;
        b.x = x + a.x;
        b.y = y + a.y;
        b.z = z + a.z;
        return b;
    }

    vec3 operator - (vec3 const &a)
    {
        vec3 b;
        b.x = x - a.x;
        b.y = y - a.y;
        b.z = z - a.z;
        return b;

    }

    double operator * (vec3 const &a)
    {
        double b;
        b = x * a.x + y * a.y + z * a.z;
        return b;
    }

    vec3 operator * (double c)
    {
        vec3 b;
        b.x = c * x;
        b.y = c * y;
        b.z = c * z;
        return b;
    }

    vec3 operator / (double c)
    {
        vec3 b;
        b.x = x / c;
        b.y = y / c;
        b.z = z / c;
        return b;
    }

    double norm()
    {
        return sqrt(x*x + y*y + z*z);
    }

    void print()
    {
        std::cout << id << ": (" << x << ", " << y << ", " << z << ")' \n";
    }
};

bool compare_complex(std::complex<double> a, std::complex<double> b)
{
    return a != b ? a.real() < b.real() : a.imag() < b.imag();
}

double smallest_positive_real_soln(std::complex<double> roots[4])
{
    double soln = SHOT_IMPOSSIBLE;

    for(int i = 0; i < 4; i++)
    {
        double imag_order = log10(abs(roots[i].imag()));
        if((imag_order < OoM || std::isnan(imag_order)) && roots[i].real() > 0)
        {
            soln = roots[i].real();
            break;
        }
    }

    return soln;
}

void pitch_yaw_gravity_model_movingtarget_const_v(vec3 Pos, vec3 Vel, vec3 G, double time_delay, double* pitch, double* yaw, bool* impossible)
{
    vec3 P_d, aim_vector, aim_uv;
    std::complex<double> coeffs[5] = {}, roots[4] = {};
    double t_sol = 0;

    P_d = Pos + Vel * time_delay;

    coeffs[0] = (G * G) * 0.25;
    coeffs[1] = Vel * G;
    coeffs[2] = P_d * G + Vel * Vel - pow(MUZZLE_VELOCITY, 2);
    coeffs[3] = (P_d * Vel) * 2;
    coeffs[4] = P_d * P_d;

    solve_quartic(coeffs, roots);
    std::sort(roots, roots+4, compare_complex);
    t_sol = smallest_positive_real_soln(roots);

    if (t_sol == SHOT_IMPOSSIBLE)
    {
        *impossible = true;
    }
    else
    {
        *impossible = false;
    }   

    aim_vector = P_d + Vel * t_sol + (G * pow(t_sol, 2)) * 0.5;
    aim_uv = aim_vector / aim_vector.norm();

    *pitch = asin(aim_uv.z);
    *yaw = acos(aim_uv.x / cos(*pitch));

    *pitch = *pitch * RAD_2_DEG;
    *yaw = *yaw * RAD_2_DEG;
}
#endif // PROJECTILE_ANGLE_CONVEL_H