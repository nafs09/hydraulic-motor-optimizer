#include "csv_utils.h"
#include <fstream>

namespace csv_utils {

    CSVWriter::CSVWriter(const std::string& filename) {
        m_file.open(filename);
    }
    CSVWriter::~CSVWriter() {
        if (m_file.is_open()) m_file.close();
    }
    bool CSVWriter::is_open() const { return m_file.is_open(); }

    void CSVWriter::write_value(double v) { m_file << v; }
    void CSVWriter::write_value(int v) { m_file << v; }

    void CSVWriter::write_row(const std::vector<double>& values) {
        if (!m_file.is_open()) return;
        for (size_t i = 0; i < values.size(); ++i) {
            if (i) m_file << ",";
            m_file << values[i];
        }
        m_file << "\n";
    }

} // namespace csv_utils
