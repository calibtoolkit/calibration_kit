#ifndef CONF_PARSER_H_
#define CONF_PARSER_H_

#include <boost/algorithm/string.hpp>
#include <fstream>
#include <iostream>
#include <map>
#include <string>

typedef std::map<std::string, std::string> KeyValues;

template<typename T>
T inline string_to_num(const std::string &str) {
    std::istringstream ss(str);
    T value;
    ss >> value;
    return value;
}


bool inline string_to_bool(const std::string &str) {
    return (str == "true" || str == "TRUE" || str == "1");
}

class ConfParser {
public:
    ConfParser() {}

    ~ConfParser() {}

    bool read_config(const std::string &filename) {
        conf_file_name_ = filename;
        conf_data_.clear();
        std::ifstream infile(filename.c_str());
        if (!infile) {
            std::cerr << "conf file open error: " << filename << std::endl;
            return false;
        }

        std::string line;
        std::string key;
        std::string value;

        bool section_start = false;
        while (std::getline(infile, line)) {
            if (0 < line.length() && get_key_and_val(line, key, value)) {
                if (value[value.size() - 1] == '\r') {
                    value[value.size() - 1] = '\0';
                }
                conf_data_[key] = value;
            }
        }

        infile.close();
        return true;
    }

    void print_config() {
        for (auto iter = conf_data_.begin(); iter != conf_data_.end(); ++iter) {
            std::cout << iter->first << "=" << iter->second << std::endl;
        }
        std::cout << std::endl;
    }

    KeyValues &get_config() {
        return conf_data_;
    }

private:

    bool get_key_and_val(const std::string &line, std::string &key,
                         std::string &val) {
        if (line.empty()) {
            return false;
        }

        int start_pos = 0;
        int end_pos = line.size() - 1;
        int pos = 0;

        if ((pos = line.find('=')) == -1) {
            return false;
        }
        key = line.substr(0, pos);
        val = line.substr(pos + 1, end_pos - pos + 1);

        boost::trim_right(key);
        boost::trim_right(val);

        if (key.empty()) {
            return false;
        }

        if (val.empty()) {
            return false;
        }

        return true;
    }


private:
    std::string conf_file_name_;
    KeyValues conf_data_;
};


#endif