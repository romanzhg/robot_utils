#include "utils.h"

#include <iostream>
#include <fstream>

#include "common_types.h"

using namespace std;

namespace mapf {
void StrSplit(const string &str, char delim, vector<string> &result) {
  size_t cur = 0;
  size_t pos = str.find(delim);
  while (pos != string::npos) {
    if (pos != cur) {
      result.push_back(str.substr(cur, pos - cur));
    }
    pos++;
    cur = pos;
    pos = str.find(delim, pos);
  }
  if (cur != str.length()) {
    result.push_back(str.substr(cur));
  }
}

vector<string> StrSplit(const string &str, char delim) {
  vector<string> rtn;
  StrSplit(str, delim, rtn);
  return rtn;
}

vector<string> GetLinesOfFile(const string &file_name) {
  ifstream input(file_name);
  if (!input) {
    cout << "Cannot open file: " << file_name << endl;
    exit(0);
  }

  vector<string> rtn;
  string tmp_line;
  while (getline(input, tmp_line)) {
    if (tmp_line[0] == '#') {
      // Skip comments.
      continue;
    }
    rtn.push_back(tmp_line);
  }
  input.close();

  return rtn;
}

bool StrStartsWith(const std::string &a, const std::string &b) {
  return a.find(b) == 0;
}

}