#include <random>

#include <spdlog/spdlog.h>

#include "Labs/1-Drawing2D/tasks.h"

using VCX::Labs::Common::ImageRGB;

namespace VCX::Labs::Drawing2D {
    /******************* 1.Image Dithering *****************/
    void DitheringThreshold(
        ImageRGB &       output,
        ImageRGB const & input) {
        for (std::size_t x = 0; x < input.GetSizeX(); ++x)
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                glm::vec3 color = input[{ x, y }];
                output.SetAt({ x, y }, {
                                           color.r > 0.5 ? 1 : 0,
                                           color.g > 0.5 ? 1 : 0,
                                           color.b > 0.5 ? 1 : 0,
                                       });
            }
    }

    void DitheringRandomUniform(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:
        std::uniform_real_distribution<float> u(-0.5, 0.5);
        std::default_random_engine            e(time(NULL));
        for (std::size_t x = 0; x < input.GetSizeX(); ++x) {
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                glm::vec3 color = input[{ x, y }];
                color += u(e);
                output.SetAt({ x, y }, {
                                           color.r > 0.5 ? 1 : 0,
                                           color.g > 0.5 ? 1 : 0,
                                           color.b > 0.5 ? 1 : 0,
                                       });
            }
        }
    }

    void DitheringRandomBlueNoise(
        ImageRGB &       output,
        ImageRGB const & input,
        ImageRGB const & noise) {
        // your code here:
        for (std::size_t x = 0; x < input.GetSizeX(); ++x) {
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                glm::vec3 color_input = input[{ x, y }];
                glm::vec3 color_noise = noise[{ x, y }];
                glm::vec3 color       = (color_input + color_noise);
                output.SetAt({ x, y }, {
                                           color.r > 1 ? 1 : 0,
                                           color.g > 1 ? 1 : 0,
                                           color.b > 1 ? 1 : 0,
                                       });
            }
        }
    }

    void DitheringOrdered(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:
        int draw_map[9][2] {
            { 1,  1 },
            { 0,  1 },
            { 1,  2 },
            { 2,  1 },
            { 2,  0 },
            { 0,  2 },
            { 0,  0 },
            { 2,  2 },
            { 1,  0 }
        };
        for (std::size_t x = 0; x < input.GetSizeX(); ++x) {
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                glm::vec3 color = input[{ x, y }];
                for (std::size_t i = 0; i < 9; ++i) {
                    std::size_t nx = 3 * x + draw_map[i][0];
                    std::size_t ny = 3 * y + draw_map[i][1];
                    if (color.r > i * 1.0 / 9) {
                        output.SetAt({ nx, ny }, { 1, 1, 1 });
                    } else {
                        output.SetAt({ nx, ny }, { 0, 0, 0 });
                    }
                }
            }
        }
    }

    void DitheringErrorDiffuse(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:
        output = input;
        for (std::size_t y = 0; y < output.GetSizeY(); ++y) {
            for (std::size_t x = 0; x < output.GetSizeX(); ++x) {
                glm::vec3 color = output[{ x, y }];
                float     val   = color.r > 0.5 ? 1 : 0;
                float     err   = color.r - val;
                if (x > 0 && y < output.GetSizeY() - 1) {
                    glm::vec3 new_color = output[{ x - 1, y + 1 }];
                    new_color += err * 3.0 / 16;
                    output.SetAt({ x - 1, y + 1 }, new_color);
                }
                if (y < output.GetSizeY() - 1) {
                    glm::vec3 new_color = output[{ x, y + 1 }];
                    new_color += err * 5.0 / 16;
                    output.SetAt({ x, y + 1 }, new_color);
                }
                if (x < output.GetSizeX() - 1 && y < output.GetSizeY() - 1) {
                    glm::vec3 new_color = output[{ x + 1, y + 1 }];
                    new_color += err * 1.0 / 16;
                    output.SetAt({ x + 1, y + 1 }, new_color);
                }
                if (x < output.GetSizeX() - 1) {
                    glm::vec3 new_color = output[{ x + 1, y }];
                    new_color += err * 7.0 / 16;
                    output.SetAt({ x + 1, y }, new_color);
                }
                output.SetAt({ x, y }, { val, val, val });
            }
        }
    }

    /******************* 2.Image Filtering *****************/
    void Blur(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:
        // Uniform Kernel
        float kernel[3][3] = {
            {1.0 / 9, 1.0 / 9, 1.0 / 9},
            {1.0 / 9, 1.0 / 9, 1.0 / 9},
            {1.0 / 9, 1.0 / 9, 1.0 / 9},
        };
        /* Gaussian Kernel
        float kernel[3][3] = {
            {1.0 / 16, 2.0 / 16, 1.0 / 16},
            {2.0 / 16, 4.0 / 16, 2.0 / 16},
            {1.0 / 16, 2.0 / 16, 1.0 / 16},
        };
        */
        auto f = [&input](std::size_t x, std::size_t y, std::size_t dx, std::size_t dy) -> bool {
            return ((x + dx >= 1) && (y + dy >= 1) && (x + dx <= input.GetSizeX()) && (y + dy <= input.GetSizeY()));
        };
        for (std::size_t x = 0; x < input.GetSizeX(); ++x) {
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                float sum_r = 0;
                float sum_g = 0;
                float sum_b = 0;
                for (std::size_t i = 0; i < 3; ++i) {
                    for (std::size_t j = 0; j < 3; ++j) {
                        if (f(x, y, i, j)) {
                            glm::vec3 color = input[{ x + i - 1, y + j - 1 }];
                            sum_r += color.r * kernel[i][j];
                            sum_g += color.g * kernel[i][j];
                            sum_b += color.b * kernel[i][j];
                        }
                    }
                }
                output.SetAt({ x, y }, { sum_r, sum_g, sum_b });
            }
        }
    }

    void Edge(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:
        float kernel_x[3][3] = {
            {-1, 0, 1},
            {-2, 0, 2},
            {-1, 0, 1}
        };
        float kernel_y[3][3] = {
            { 1,  2,  1},
            { 0,  0,  0},    
            {-1, -2, -1}
        };
        auto f = [&input](std::size_t x, std::size_t y, std::size_t dx, std::size_t dy) -> bool {
            return ((x + dx >= 1) && (y + dy >= 1) && (x + dx <= input.GetSizeX()) && (y + dy <= input.GetSizeY()));
        };
        for (std::size_t x = 0; x < input.GetSizeX(); ++x) {
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                float sum_r_x = 0, sum_r_y = 0;
                float sum_g_x = 0, sum_g_y = 0;
                float sum_b_x = 0, sum_b_y = 0;
                for (std::size_t i = 0; i < 3; ++i) {
                    for (std::size_t j = 0; j < 3; ++j) {
                        if (f(x, y, i, j)) {
                            glm::vec3 color = input[{ x + i - 1, y + j - 1 }];
                            sum_r_x += color.r * kernel_x[i][j];
                            sum_r_y += color.r * kernel_y[i][j];
                            sum_g_x += color.g * kernel_x[i][j];
                            sum_g_y += color.g * kernel_y[i][j];
                            sum_b_x += color.b * kernel_x[i][j];
                            sum_b_y += color.b * kernel_y[i][j];
                        }
                    }
                }
                float grad_r = sqrt(sum_r_x * sum_r_x + sum_r_y * sum_r_y);
                float grad_g = sqrt(sum_g_x * sum_g_x + sum_g_y * sum_g_y);
                float grad_b = sqrt(sum_b_x * sum_b_x + sum_b_y * sum_b_y);
                output.SetAt({ x, y }, { grad_r , grad_g , grad_b });
            }
        }
    }

    /******************* 3. Image Inpainting *****************/
    void Inpainting(
        ImageRGB &         output,
        ImageRGB const &   inputBack,
        ImageRGB const &   inputFront,
        const glm::ivec2 & offset) {
        output             = inputBack;
        size_t      width  = inputFront.GetSizeX();
        size_t      height = inputFront.GetSizeY();
        glm::vec3 * g      = new glm::vec3[width * height];
        memset(g, 0, sizeof(glm::vec3) * width * height);
        // set boundary condition
        for (std::size_t y = 0; y < height; ++y) {
            // set boundary for (0, y), your code: g[y * width] = ?
            std::size_t bx = offset.x, by = offset.y + y;
            g[y * width] = inputBack[{ bx, by }] - inputFront[{ 0, y }];
            // set boundary for (width - 1, y), your code: g[y * width + width - 1] = ?
            bx = offset.x + width - 1;
            g[y * width + width - 1] = inputBack[{ bx, by }] - inputFront[{ width - 1, y }];
        }
        for (std::size_t x = 0; x < width; ++x) {
            // set boundary for (x, 0), your code: g[x] = ?
            std::size_t bx = offset.x + x, by = offset.y;
            g[x] = inputBack[{ bx, by }] - inputFront[{ x, 0 }];
            // set boundary for (x, height - 1), your code: g[(height - 1) * width + x] = ?
            by = offset.y + height - 1;
            g[(height - 1) * width + x] = inputBack[{ bx, by }] - inputFront[{ x, height - 1 }];
        }

        // Jacobi iteration, solve Ag = b
        for (int iter = 0; iter < 8000; ++iter) {
            for (std::size_t y = 1; y < height - 1; ++y)
                for (std::size_t x = 1; x < width - 1; ++x) {
                    g[y * width + x] = (g[(y - 1) * width + x] + g[(y + 1) * width + x] + g[y * width + x - 1] + g[y * width + x + 1]);
                    g[y * width + x] = g[y * width + x] * glm::vec3(0.25);
                }
        }

        for (std::size_t y = 0; y < inputFront.GetSizeY(); ++y)
            for (std::size_t x = 0; x < inputFront.GetSizeX(); ++x) {
                glm::vec3 color = g[y * width + x] + inputFront.GetAt({ x, y });
                output.SetAt({ x + offset.x, y + offset.y }, color);
            }
        delete[] g;
    }

    /******************* 4. Line Drawing *****************/
    void DrawLine(
        ImageRGB &       canvas,
        glm::vec3 const  color,
        glm::ivec2 const p0,
        glm::ivec2 const p1) {
        // your code here:
        int dx = 2 * abs(p0.x - p1.x);
        int dy = 2 * abs(p0.y - p1.y);
        int sign = (p0.x - p1.x) * (p0.y - p1.y);
        sign     = sign >= 0 ? 1 : -1;
        if (dy <= dx) {
            int         dydx    = dy - dx;
            int         F       = dy - dx / 2;
            std::size_t y       = 0;
            if ((p0.y - p1.y) * sign >= 0) y = p1.y;
            else y = p0.y;
            std::size_t x_start = p0.x > p1.x ? p1.x : p0.x;
            std::size_t x_end   = p0.x + p1.x - x_start;
            for (std::size_t x = x_start; x <= x_end; ++x) {
                canvas.SetAt({ x, y }, color);
                if (F < 0) F += dy;
                else {
                    y += sign;
                    F += dydx;
                }
            }
        } else {
            int dxdy = dx - dy;
            int F    = dx - dy / 2;
            std::size_t x    = 0;
            if ((p0.x - p1.x) * sign >= 0) x = p1.x;
            else x = p0.x;
            std::size_t y_start = p0.y > p1.y ? p1.y : p0.y;
            std::size_t y_end   = p0.y + p1.y - y_start;
            for (std::size_t y = y_start; y <= y_end; ++y) {
                canvas.SetAt({ x, y }, color);
                if (F < 0) F += dx;
                else {
                    x += sign;
                    F += dxdy;
                }
            }
        }
    }

    /******************* 5. Triangle Drawing *****************/
    void DrawTriangleFilled(
        ImageRGB &       canvas,
        glm::vec3 const  color,
        glm::ivec2 const p0,
        glm::ivec2 const p1,
        glm::ivec2 const p2) {
        // your code here:
        auto DrawFlatTriangleFilled = [&canvas, &color](
                                          glm::ivec2 const pp1,
                                          glm::ivec2 const pp2,
                                          glm::ivec2 const pp0)
            -> void {
            // This function should have been implemented outside `DrawTriangleFilled()`,
            // but as there is a "your code here" instruction, I just put it here as a lambda function.
            // pp0 is the point whose y is different to the other two.
            float k1 = FLT_MAX, k2 = FLT_MAX;
            float b1 = 0, b2 = 0;
            if (pp1.x != pp0.x) {
                k1 = (pp1.y - pp0.y) * 1.0 / (pp1.x - pp0.x);
                b1 = pp0.y - k1 * pp0.x;
            }
            if (pp2.x != pp0.x) {
                k2 = (pp2.y - pp0.y) * 1.0 / (pp2.x - pp0.x);
                b2 = pp0.y - k2 * pp0.x;
            }
            std::size_t y_start = pp0.y > pp1.y ? pp1.y : pp0.y;
            std::size_t y_end   = pp0.y > pp1.y ? pp0.y : pp1.y;
            for (std::size_t y = y_start; y < y_end; ++y) {
                float x1 = (k1 == FLT_MAX) ? pp0.x : (y - b1) / k1;
                float x2 = (k2 == FLT_MAX) ? pp0.x : (y - b2) / k2;
                glm::ivec2 pt1 = { round(x1), round(y) };
                glm::ivec2 pt2 = { round(x2), round(y) };
                DrawLine(canvas, color, pt1, pt2);
            }
        };
        glm::ivec2 pmax = p0.y > p1.y ? p0 : p1;
        pmax            = pmax.y > p2.y ? pmax : p2;
        glm::ivec2 pmin = p0.y > p1.y ? p1 : p0;
        pmin            = pmin.y > p2.y ? p2 : pmin;
        glm::ivec2 pmid = p0 + p1 + p2 - pmax - pmin;
        if (pmax.y == pmid.y) DrawFlatTriangleFilled(pmax, pmid, pmin);
        else if (pmin.y == pmid.y) DrawFlatTriangleFilled(pmin, pmid, pmax);
        else {
            float k = FLT_MAX;
            float b = 0;
            if (pmax.x != pmin.x) {
                k = (pmax.y - pmin.y) * 1.0 / (pmax.x - pmin.x);
                b = pmax.y - k * pmax.x;
            }
            glm::ivec2 pref = { 0, 0 };
            pref.y          = pmid.y;
            pref.x          = (k == FLT_MAX) ? pmin.x : (pmid.y - b) / k;
            DrawFlatTriangleFilled(pref, pmid, pmax);
            DrawFlatTriangleFilled(pref, pmid, pmin);
        }
        DrawLine(canvas, color, p0, p1);
        DrawLine(canvas, color, p1, p2);
        DrawLine(canvas, color, p2, p0);
    }

    /******************* 6. Image Supersampling *****************/
    void Supersample(
        ImageRGB &       output,
        ImageRGB const & input,
        int              rate) {
        // your code here:
        std::size_t w = output.GetSizeX(),
                    h = output.GetSizeY();
        std::size_t wi = input.GetSizeX(),
                    hi = input.GetSizeY();
        float size_rate_x = wi * 1.0 / w,
              size_rate_y = hi * 1.0 / h;
        for (std::size_t x = 0; x < w; ++x) {
            for (std::size_t y = 0; y < h; ++y) {
                float nx = x * size_rate_x,
                      ny = y * size_rate_y;
                glm::vec3 tot = { 0, 0, 0 };
                for (std::size_t i = 0; i < rate; ++i) {
                    for (std::size_t j = 0; j < rate; ++j) {
                        float nnx = nx + i * 1.0 / rate * size_rate_x;
                        float nny = ny + j * 1.0 / rate * size_rate_y;
                        tot += input.GetAt({ (std::size_t)nnx, (std::size_t)nny });
                    }
                }
                tot /= (rate * rate);
                output.SetAt({ x, y }, tot);
            }
        }

    }

    /******************* 7. Bezier Curve *****************/
    glm::vec2 CalculateBezierPoint(
        std::span<glm::vec2> points,
        float const          t) {
        // your code here:
        if (points.size() == 2) {
            return (1 - t) * points[0] + t * points[1];
        } else if (points.size() > 2) {
            std::vector<glm::vec2> new_points;
            for (std::size_t i = 0; i < points.size() - 1; ++i) {
                new_points.push_back((1 - t) * points[i] + t * points[i + 1]);
            }
            std::span<glm::vec2> new_points_span { new_points };
            return CalculateBezierPoint(new_points_span, t);
        }
        return glm::vec2 {0, 0};
    }
} // namespace VCX::Labs::Drawing2D