#include <string>
#include <iostream>
#include <experimental/filesystem>

using namespace std;
int main()
{
  std::string path = "../data/mapf_map";
  for (const auto & entry : experimental::filesystem::directory_iterator(path)) {
    std::cout << entry.path() << std::endl;
    std::cout << entry.path().filename() << endl;
    std::cout << entry.path().parent_path() << std::endl;
    std::cout << entry.path().root_directory() << std::endl;

//    std::cout << entry.path().relative_path() << std::endl;
//    std::cout << entry.path().generic_string() << std::endl;
//    std::cout << entry.path().filename() << std::endl;
//    std::cout << entry.path().parent_path() << std::endl;
//    std::cout << entry.path().root_path() << std::endl;
//    std::cout << entry.path().root_directory() << std::endl;
//    std::cout << entry.path().root_name() << std::endl;
    cout << endl;
  }

  return 0;
}
