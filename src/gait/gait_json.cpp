#include <ctype.h>
#include <stdlib.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

bool gait_read_text_file(const std::string &path, std::string *content, std::string *error)
{
    std::ifstream input(path.c_str());
    if (!input)
    {
        if (error != 0)
        {
            *error = "failed to open file: " + path;
        }
        return false;
    }

    std::stringstream buffer;
    buffer << input.rdbuf();
    *content = buffer.str();
    return true;
}

static size_t skip_ws(const std::string &text, size_t pos)
{
    while (pos < text.size() && isspace((unsigned char)text[pos]))
    {
        pos++;
    }
    return pos;
}

static bool find_field_value_start(const std::string &json, const std::string &field, size_t *value_start)
{
    std::string needle = "\"" + field + "\"";
    size_t pos = json.find(needle);
    if (pos == std::string::npos)
    {
        return false;
    }

    pos = json.find(':', pos + needle.size());
    if (pos == std::string::npos)
    {
        return false;
    }

    *value_start = skip_ws(json, pos + 1);
    return true;
}

bool gait_json_get_string(const std::string &json, const std::string &field, std::string *value)
{
    size_t pos = 0;
    if (!find_field_value_start(json, field, &pos) || pos >= json.size() || json[pos] != '"')
    {
        return false;
    }

    pos++;
    std::string result;
    while (pos < json.size())
    {
        char c = json[pos++];
        if (c == '"')
        {
            *value = result;
            return true;
        }
        if (c == '\\' && pos < json.size())
        {
            char escaped = json[pos++];
            switch (escaped)
            {
                case '"':
                case '\\':
                case '/':
                    result.push_back(escaped);
                    break;
                case 'n':
                    result.push_back('\n');
                    break;
                case 't':
                    result.push_back('\t');
                    break;
                default:
                    result.push_back(escaped);
                    break;
            }
        }
        else
        {
            result.push_back(c);
        }
    }

    return false;
}

bool gait_json_get_int(const std::string &json, const std::string &field, int *value)
{
    size_t pos = 0;
    if (!find_field_value_start(json, field, &pos))
    {
        return false;
    }

    char *end = 0;
    long parsed = strtol(json.c_str() + pos, &end, 10);
    if (end == json.c_str() + pos)
    {
        return false;
    }

    *value = (int)parsed;
    return true;
}

static bool extract_array_bounds(const std::string &json, const std::string &field, size_t *start, size_t *end)
{
    size_t pos = 0;
    if (!find_field_value_start(json, field, &pos) || pos >= json.size() || json[pos] != '[')
    {
        return false;
    }

    int depth = 0;
    bool in_string = false;
    for (size_t i = pos; i < json.size(); i++)
    {
        char c = json[i];
        if (c == '"' && (i == 0 || json[i - 1] != '\\'))
        {
            in_string = !in_string;
        }
        if (in_string)
        {
            continue;
        }
        if (c == '[')
        {
            depth++;
        }
        else if (c == ']')
        {
            depth--;
            if (depth == 0)
            {
                *start = pos + 1;
                *end = i;
                return true;
            }
        }
    }

    return false;
}

std::vector<std::string> gait_json_get_object_array(const std::string &json, const std::string &field)
{
    std::vector<std::string> objects;
    size_t start = 0;
    size_t end = 0;
    if (!extract_array_bounds(json, field, &start, &end))
    {
        return objects;
    }

    bool in_string = false;
    int depth = 0;
    size_t object_start = std::string::npos;
    for (size_t i = start; i < end; i++)
    {
        char c = json[i];
        if (c == '"' && (i == 0 || json[i - 1] != '\\'))
        {
            in_string = !in_string;
        }
        if (in_string)
        {
            continue;
        }
        if (c == '{')
        {
            if (depth == 0)
            {
                object_start = i;
            }
            depth++;
        }
        else if (c == '}')
        {
            depth--;
            if (depth == 0 && object_start != std::string::npos)
            {
                objects.push_back(json.substr(object_start, i - object_start + 1));
                object_start = std::string::npos;
            }
        }
    }

    return objects;
}

std::vector<std::string> gait_json_get_string_array(const std::string &json, const std::string &field)
{
    std::vector<std::string> values;
    size_t start = 0;
    size_t end = 0;
    if (!extract_array_bounds(json, field, &start, &end))
    {
        return values;
    }

    size_t pos = start;
    while (pos < end)
    {
        pos = skip_ws(json, pos);
        if (pos >= end)
        {
            break;
        }
        if (json[pos] == ',')
        {
            pos++;
            continue;
        }
        if (json[pos] != '"')
        {
            values.clear();
            return values;
        }

        pos++;
        std::string value;
        bool complete = false;
        while (pos < end)
        {
            char c = json[pos++];
            if (c == '"')
            {
                complete = true;
                break;
            }
            if (c == '\\' && pos < end)
            {
                value.push_back(json[pos++]);
            }
            else
            {
                value.push_back(c);
            }
        }
        if (!complete)
        {
            values.clear();
            return values;
        }
        values.push_back(value);
    }

    return values;
}
