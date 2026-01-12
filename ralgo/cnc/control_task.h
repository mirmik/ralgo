#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <igris/container/static_vector.h>
#include <nos/shell/argv.h>
#include <ralgo/cnc/defs.h>
#include <ralgo/cnc/error_handler.h>
#include <ralgo/cnc/util.h>
#include <ralgo/linalg/vector_view.h>

namespace cnc
{
    /// Parse double with correct scientific notation support
    /// (igris_atof64 has a bug with negative exponents)
    static inline double parse_double(const char* str, char** endptr = nullptr)
    {
        if (!str) return 0.0;

        char* end = nullptr;
        double mantissa = 0.0;
        int exponent = 0;
        int sign = 1;
        int exp_sign = 1;

        // Skip whitespace
        while (*str == ' ' || *str == '\t') str++;

        // Parse sign
        if (*str == '-') { sign = -1; str++; }
        else if (*str == '+') { str++; }

        // Parse mantissa integer part
        while (*str >= '0' && *str <= '9') {
            mantissa = mantissa * 10.0 + (*str - '0');
            str++;
        }

        // Parse decimal part
        if (*str == '.') {
            str++;
            double decimal_place = 0.1;
            while (*str >= '0' && *str <= '9') {
                mantissa += (*str - '0') * decimal_place;
                decimal_place *= 0.1;
                str++;
            }
        }

        // Parse exponent
        if (*str == 'e' || *str == 'E') {
            str++;
            if (*str == '-') { exp_sign = -1; str++; }
            else if (*str == '+') { str++; }

            while (*str >= '0' && *str <= '9') {
                exponent = exponent * 10 + (*str - '0');
                str++;
            }
        }

        if (endptr) *endptr = const_cast<char*>(str);

        double result = sign * mantissa;
        if (exponent != 0) {
            result *= std::pow(10.0, exp_sign * exponent);
        }
        return result;
    }
    struct idxpos
    {
        int idx;
        cnc_float_type pos;
    };

    class control_task
    {
    public:
        bool isok = false;
        igris::static_vector<cnc_float_type, NMAX_AXES> _poses = {};
        cnc_float_type feed = 0;
        cnc_float_type acc = 0;
        std::vector<size_t> active_axes;

        control_task(size_t total_axes)
        {
            _poses.resize(total_axes);
            // Явно инициализируем нулями для надёжности
            for (size_t i = 0; i < total_axes; ++i)
            {
                _poses[i] = 0;
            }
        }

        control_task(const control_task &other)
        {
            isok = other.isok;
            feed = other.feed;
            acc = other.acc;
            _poses = other._poses;
        }

        void set_active_axes(const std::vector<size_t> &axes)
        {
            active_axes = axes;
        }

        void set_active_axes(const std::vector<idxpos> &axes)
        {
            active_axes.clear();
            for (auto &pair : axes)
            {
                active_axes.push_back(pair.idx);
            }
        }

        control_task &operator=(const control_task &other)
        {
            isok = other.isok;
            feed = other.feed;
            acc = other.acc;
            _poses = other._poses;
            return *this;
        }

        igris::static_vector<cnc_float_type, NMAX_AXES> &poses()
        {
            return _poses;
        }

        const igris::static_vector<cnc_float_type, NMAX_AXES> &poses() const
        {
            return _poses;
        }

        void set_poses(const igris::array_view<cnc_float_type> &arr)
        {
            std::copy(arr.begin(), arr.end(), _poses.begin());
        }

        int parse(const nos::argv &argv)
        {
            for (unsigned int i = 0; i < argv.size(); ++i)
            {
                char symb = argv[i].data()[0];
                cnc_float_type val = atof(&argv[i].data()[1]);

                switch (tolower(symb))
                {
                case 'f':
                    feed = val;
                    continue;
                case 'm':
                    acc = val;
                    continue;
                case 'x':
                    _poses[0] = val;
                    continue;
                case 'y':
                    _poses[1] = val;
                    continue;
                case 'z':
                    _poses[2] = val;
                    continue;
                case 'a':
                    _poses[3] = val;
                    continue;
                case 'b':
                    _poses[4] = val;
                    continue;
                case 'c':
                    _poses[5] = val;
                    continue;
                case 'i':
                    _poses[6] = val;
                    continue;
                case 'j':
                    _poses[7] = val;
                    continue;
                case 'k':
                    _poses[8] = val;
                    continue;
                default:
                    return -1;
                }
            }
            return 0;
        }

        std::string to_string() const
        {
            std::string ret;
            ret += "control_task(";
            ret += "isok=";
            ret += isok ? "true" : "false";
            ret += "feed=";
            ret += std::to_string(feed);
            ret += ", acc=";
            ret += std::to_string(acc);
            ret += ", poses=[";
            for (size_t i = 0; i < _poses.size() - 1; ++i)
            {
                ret += std::to_string(_poses[i]);
                ret += ", ";
            }
            ret += std::to_string(_poses[_poses.size() - 1]);
            ret += "])";
            return ret;
        }
    };

    static inline std::vector<idxpos>
    get_task_poses_from_argv(const nos::argv &argv, error_handler* err_handler = nullptr)
    {
        std::vector<idxpos> ret;
        for (unsigned int i = 0; i < argv.size(); ++i)
        {
            char symb = tolower(argv[i].data()[0]);
            int idx = symbol_to_index(symb);
            if (idx >= 0)
            {
                cnc_float_type val = parse_double(&argv[i].data()[1], nullptr);
                // Диагностика: проверяем что парсинг не дал мусор
                if (err_handler && (std::isnan(val) || std::isinf(val) || std::abs(val) > 1e10))
                {
                    // ctx=200+idx: плохое значение при парсинге оси idx
                    err_handler->report(error_code::timing_overflow, static_cast<int8_t>(idx), 200 + idx);
                }
                ret.push_back({idx, val});
            }
        }
        return ret;
    }

    static inline cnc_float_type get_task_feed_from_argv(const nos::argv &argv)
    {
        for (unsigned int i = 0; i < argv.size(); ++i)
        {
            char symb = tolower(argv[i].data()[0]);
            if (symb == 'f')
            {
                cnc_float_type val = parse_double(&argv[i].data()[1], nullptr);
                return val;
            }
        }
        return 0;
    }

    static inline cnc_float_type get_task_acc_from_argv(const nos::argv &argv)
    {
        for (unsigned int i = 0; i < argv.size(); ++i)
        {
            char symb = tolower(argv[i].data()[0]);
            if (symb == 'm')
            {
                cnc_float_type val = parse_double(&argv[i].data()[1], nullptr);
                return val;
            }
        }
        return 0;
    }

}

#endif