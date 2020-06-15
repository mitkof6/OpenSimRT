// Read an INI file into easy-to-access name/value pairs.

// inih and INIReader are released under the New BSD license (see LICENSE.txt).
// Go to the project home page for more info:
//
// http://code.google.com/p/inih/

#pragma warning(disable : 4251)

#ifndef INI_READER_H
#    define INI_READER_H

#    include "internal/CommonExports.h"

#    include <map>
#    include <string>
#    include <type_traits>
#    include <vector>

// Read an INI file into easy-to-access name/value pairs. (Note that I've gone
// for simplicity here rather than speed, but it should be pretty decent.)
class Common_API INIReader {
 public:
    INIReader(){};
    // Construct INIReader and parse given filename. See ini.h for more info
    // about the parsing.
    INIReader(std::string filename);
    // Return the result of ini_parse(), i.e., 0 on success, line number of
    // first error on parse error, or -1 on file open error.
    int parseError();
    // Get a string value from INI file, returning default_value if not found.
    std::string getString(std::string section, std::string name,
                          std::string default_value);
    // Get an integer (long) value from INI file, returning default_value if not
    // found or not a valid integer (decimal "1234", "-1234", or hex "0x4d2").
    long getInteger(std::string section, std::string name, long default_value);
    // Get a real (floating point double) value from INI file, returning
    // default_value if not found or not a valid floating point value according
    // to strtod().
    double getReal(std::string section, std::string name, double default_value);
    // Get a boolean value from INI file, returning default_value if not found
    // or if not a valid true/false value. Valid true values are "true", "yes",
    // "on", "1", and valid false values are "false", "no", "off", "0" (not case
    // sensitive).
    bool getBoolean(std::string section, std::string name, bool default_value);

    template <typename T>
    std::vector<T> getVector(std::string section, std::string name,
                             std::vector<T> default_value) {
        std::string key = makeKey(section, name);

        auto separate = [](std::string str, std::string&& delimiter) {
            std::vector<T> result;
            std::string token;
            size_t pos = 0;
            str += delimiter; // to get the last token
            while ((pos = str.find(delimiter)) != std::string::npos) {
                token = str.substr(0, pos);
                if constexpr (std::is_same<T, std::string>::value) {
                    result.push_back(token);
                } else if constexpr (std::is_same<T, int>::value) {
                    const char* value = token.c_str();
                    char* end;
                    result.push_back(strtol(value, &end, 0));
                } else if constexpr (std::is_same<T, double>::value) {
                    const char* value = token.c_str();
                    char* end;
                    result.push_back(strtod(value, &end));
                }
                str.erase(0, pos + delimiter.length());
            }
            return result;
        };
        return _values.count(key) ? separate(_values[key], " ") : default_value;
    }

 private:
    std::map<std::string, std::string> _values;
    int _error;
    static std::string makeKey(std::string section, std::string name);
    static int valueHandler(void* user, const char* section, const char* name,
                            const char* value);
    /* Parse given INI-style file. May have [section]s, name=value pairs
       (whitespace stripped), and comments starting with ';'
       (semicolon). Section is "" if name=value pair parsed before any
       section heading. name:value pairs are also supported as a concession
       to Python's ConfigParser.

       For each name=value pair parsed, call handler function with given
       user pointer as well as section, name, and value (data only valid for
       duration of handler call). Handler should return nonzero on success,
       zero on error.

       Returns 0 on success, line number of first error on parse error
       (doesn't stop on first error), -1 on file open error, or -2 on memory
       allocation error (only when INI_USE_STACK is zero).
    */
    int ini_parse(const char* filename,
                  int (*handler)(void* user, const char* section,
                                 const char* name, const char* value),
                  void* user);
    /* Same as ini_parse(), but takes a FILE* instead of filename. This
       doesn't
       close the file when it's finished -- the caller must do that. */
    int ini_parse_file(FILE* file,
                       int (*handler)(void* user, const char* section,
                                      const char* name, const char* value),
                       void* user);
};

#endif
