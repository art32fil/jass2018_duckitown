#include <string>
#include <vector>
#include <iostream>
#include <sstream>

std::vector<int> extract_coordinates(std::string& str)
{
	std::vector<int> coordinates;
	std::stringstream ss(str);
	std::string id;
	ss >> id;
	int x;
	ss >> x;
	coordinates.emplace_back(x);
	ss >> x; 
	coordinates.emplace_back(x);
	return coordinates;
}

int main()
{
	std::string str = "air 8 4";
	std::vector<int> coordinates(2);
	coordinates = extract_coordinates(str);
	for (int i = 0; i < 2; i++)
	{
		std::cout << coordinates[i];
	}
	return 0;
}