#pragma once

#include <string>
#include <vector>
#include <fstream>

namespace csv_utils {

    class CSVWriter {
    public:
        explicit CSVWriter(const std::string& filename);
        ~CSVWriter();

        CSVWriter(const CSVWriter&) = delete;
        CSVWriter& operator=(const CSVWriter&) = delete;

        CSVWriter(CSVWriter&&) = default;
        CSVWriter& operator=(CSVWriter&&) = default;

        template<typename... Args>
        void write_row(double first, Args... rest) {
            if (!m_file.is_open()) return;
            m_file << first;
            write_values(rest...);
            m_file << "\n";
        }
        void write_row(std::initializer_list<double> values) {
            std::vector<double> vec(values);
            write_row(vec);
        }

        template<typename... Args>
        void write_row(int first, Args... rest) {
            if (!m_file.is_open()) return;
            m_file << first;
            write_values(rest...);
            m_file << "\n";
        }

        void write_row(const std::vector<double>& values);

        bool is_open() const;

    private:
        void write_value(double v);
        void write_value(int v);

        template<typename... Args>
        void write_values(double first, Args... rest) {
            m_file << "," << first;
            if constexpr (sizeof...(rest) > 0) {
                write_values(rest...);
            }
        }

        template<typename... Args>
        void write_values(int first, Args... rest) {
            m_file << "," << first;
            if constexpr (sizeof...(rest) > 0) {
                write_values(rest...);
            }
        }

        std::ofstream m_file;
    };

} // namespace csv_utils
