#include <iostream>
#include <fstream>
#include <jsoncpp/json/json.h> // or jsoncpp/json.h , or json/json.h etc.
#include "../Eigen/Dense"

using namespace std;
using namespace Eigen;

template<typename Derived>
Json::Value MatrixToJson(const MatrixBase<Derived>& mat)
{
  cout << mat << endl;
  Json::Value jsonMat;
  for (int i = 0; i < mat.rows(); ++i) {
    jsonMat.append(Json::Value::null);
    for (int j = 0; j < mat.cols(); ++j) {
      jsonMat[i].append(mat(i ,j));
    }
  }
  cout << "jsonMat\n" << jsonMat << endl;
  return jsonMat;
} 

int main() {
  try {
    ifstream ifs("data.json");
    Json::Reader reader;
    Json::Value obj;
    reader.parse(ifs, obj); // reader can also read strings
    cout << "Book: " << obj["book"].asString() << endl;
    cout << "Year: " << obj.get("year", -1).asUInt() << endl;
    const Json::Value& characters = obj["characters"]; // array of characters
    for (int i = 0; i < characters.size(); i++){
        cout << "    name: " << characters[i]["name"].asString();
        cout << " chapter: " << characters[i]["chapter"].asUInt();
        cout << endl;
    }
    const Json::Value& array = obj["Array"]; // array of characters
    for (int i = 0; i < array.size(); i++){
        cout << "element " << array[i].asUInt();
        cout << endl;
    }
    
    Matrix4f testMat = Matrix4f::Identity();
    Json::Value res = MatrixToJson(testMat);
  } catch (Json::Exception& e){
    cout << "ERROR" << endl;
    cout << e.what();
  }
}
