#include "Labs/5-Visualization/tasks.h"

#include <numbers>

using VCX::Labs::Common::ImageRGB;
namespace VCX::Labs::Visualization {

    struct CoordinateStates {
        // your code here
        std::vector<Car> data;
        std::string      property_names[7] = { "cylinders", "displacement", "weight", "horsepower", "acceleration", "mileage", "year" };
        float            property_max[7]   = {};
        float            property_min[7]   = { FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX };
        float            property_delta[7] = {};
        int              shown_max[7] {};
        int              shown_min[7] {};
        CoordinateStates(std::vector<Car> const & _data):
            data(_data) {
            for (auto t : data) {
                float properties[7] = { t.cylinders, t.displacement, t.weight, t.horsepower, t.acceleration, t.mileage, t.year };
                for (int i = 0; i < 7; ++i) {
                    property_max[i] = std::max(property_max[i], properties[i]);
                    property_min[i] = std::min(property_min[i], properties[i]);
                }
            }
            for (int i = 0; i < 7; ++i) {
                property_delta[i] = std::max((property_max[i] - property_min[i]) / 10, 1.0f);
                shown_max[i]      = int(round(property_max[i] + property_delta[i]));
                shown_min[i]      = int(round(property_min[i] - property_delta[i]));
            }
            property_val = (shown_min[0] + shown_max[0]) / 2.0f;
        }
        
        int  main_property = 0;
        float property_val = 0;

        int   n_num = 7;
        float dx    = 1.0f / n_num;

        glm::vec2 rect_size  = glm::vec2(0.03, 0.7);
        float     rect_top   = 0.15;
        glm::vec4 rect_color = glm::vec4(0.5);

        bool Update(InteractProxy const& proxy) {
            if (! proxy.IsClicking()) return false;
            glm::vec2 mp = proxy.MousePos();
            for (int i = 0; i < 7; ++i) {
                if (mp[0] <= (i + 0.5f) * dx + 0.5f * rect_size[0] && mp[0] >= (i + 0.5f) * dx - 0.5f * rect_size[0]) {
                    if (mp[1] >= rect_top && mp[1] <= rect_top + rect_size[1]) {
                        main_property = i;
                        property_val  = shown_min[i] + (shown_max[i] - shown_min[i]) * (rect_top + rect_size[1] - mp[1]) / (rect_size[1]);
                        return true;
                    }
                }
            }
            return false;
        }

        void Paint(Common::ImageRGB& input) {
            SetBackGround(input, glm::vec4(1));

            // Draw parallel lines and set axis infomation
            glm::vec4 text_color      = glm::vec4(0, 0, 0, 1);
            float     text_center     = 0.05;
            float     max_center      = 0.1;
            float     min_center      = 0.9;
            float     text_lineheight = 0.02;
            for (int i = 0; i < 7; ++i) {
                DrawFilledRect(input, rect_color, glm::vec2((i + 0.5f) * dx - 0.5f * rect_size[0], rect_top), rect_size);
                PrintText(input, text_color, glm::vec2((i + 0.5f) * dx, text_center), text_lineheight, property_names[i]);
                PrintText(input, text_color, glm::vec2((i + 0.5f) * dx, max_center), text_lineheight, std::to_string(shown_max[i]));
                PrintText(input, text_color, glm::vec2((i + 0.5f) * dx, min_center), text_lineheight, std::to_string(shown_min[i]));
            }
            std::string bottom_txt = property_names[main_property] + " = " + std::to_string(int(round(property_val)));
            PrintText(input, text_color, glm::vec2(0.5, 0.95), text_lineheight, bottom_txt);

            // Draw inside lines
            auto getpos = [&](float val, int Property) -> glm::vec2 {
                float x_val = (Property + 0.5f) * dx;
                float y_val = rect_top + (shown_max[Property] - val) / (shown_max[Property] - shown_min[Property]) * rect_size[1];
                return glm::vec2(x_val, y_val);
            };
            glm::vec4 line_color_cold = glm::vec4(0, 0.2, 0.8, 0.3);
            glm::vec4 line_color_warm = glm::vec4(0.8, 0.2, 0, 0.3);
            for (auto t : data) {
                std::vector<glm::vec2> single_data = std::vector(7, glm::vec2(0));

                float properties[7] = { t.cylinders, t.displacement, t.weight, t.horsepower, t.acceleration, t.mileage, t.year };
                for (int i = 0; i < 7; ++i) {
                    single_data[i] = getpos(properties[i], i);
                }

                float lambda = 0;
                float EPS    = 1e-3;
                if (properties[main_property] > property_val) {
                    lambda = (properties[main_property] - property_val) / (shown_max[main_property] - property_val + EPS);
                } else {
                    lambda = (property_val - properties[main_property]) / (property_val - shown_min[main_property] + EPS);
                }
                glm::vec4 line_color = lambda * line_color_cold + (1 - lambda) * line_color_warm;

                for (int i = 0; i < 6; ++i) {
                    DrawLine(input, line_color, single_data[i], single_data[i + 1], 1);
                }
            }
        }
    };

    bool PaintParallelCoordinates(Common::ImageRGB & input, InteractProxy const & proxy, std::vector<Car> const & data, bool force) {
        // your code here
        // for example: 
        //   static CoordinateStates states(data);
        //   SetBackGround(input, glm::vec4(1));
        //   ...
        static CoordinateStates states(data);
        bool change = states.Update(proxy);
        if (! force && ! change) return false;
        states.Paint(input);
        return true;
    }

    void LIC(ImageRGB & output, Common::ImageRGB const & noise, VectorField2D const & field, int const & step) {
        // your code here
    }
}; // namespace VCX::Labs::Visualization