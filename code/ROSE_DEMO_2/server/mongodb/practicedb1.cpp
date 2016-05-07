#include <bsoncxx/builder/stream/document.hpp>
#include <bsoncxx/types.hpp>

#include <mongocxx/client.hpp>
#include <mongocxx/instance.hpp>
#include <mongocxx/uri.hpp>

using bsoncxx::builder::stream::document;
using bsoncxx::builder::stream::open_document;
using bsoncxx::builder::stream::close_document;
using bsoncxx::builder::stream::open_array;
using bsoncxx::builder::stream::close_array;
using bsoncxx::builder::stream::finalize;

int main() {
	mongocxx::instance inst{};
	mongocxx::client conn{};

	//this line connects to a database name "test". if the database does not exist
	//then it will automatically be created with this name
	auto db = conn["test"];

	//a document is defined by inserting key/ value pairs directly using the "<<" operator
	//the values inserted will alternate between key and value
	//for example: document{} << "hello" << "world" << "hello again" << "bitch" inserts a key value pair with
	//"hello" being the key and "world" being the value, then inserts "hello again" as a second key and "bitch" as
	//the value to the "hello again" key
	bsoncxx::document::value restaurant_doc =
    document{} << "address" << open_document << "street"
               << "2 Avenue"
               << "zipcode"
               << "10075"
               << "building"
               << "1480"
               << "coord" << open_array << -73.9557413 << 40.7720266 << close_array
               << close_document << "borough"
               << "Manhattan"
               << "cuisine"
               << "Italian"
               << "grades" << open_array << open_document << "date"
               << bsoncxx::types::b_date{12323} << "grade"
               << "A"
               << "score" << 11 << close_document << open_document << "date"
               << bsoncxx::types::b_date{121212} << "grade"
               << "B"
               << "score" << 17 << close_document << close_array << "name"
               << "Vella"
               << "restaurant_id"
               << "41704620" << finalize;

	auto cursor = collection.find();
	for (int i = 0; i <  
	
// We choose to move in our document here, which transfers ownership to insert_one()
    // with this line, we insert into a collection named "restaurants". again, if the 
   	// collection does not exist, then it will be automatically created for you
	//insert_one() take one parameter of type "document";
	auto res = db["restaurants"].insert_one(std::move(restaurant_doc));
}
