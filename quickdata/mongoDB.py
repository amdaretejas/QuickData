import pymongo

myclient = pymongo.MongoClient("mongodb+srv://TejasAmdare:IZXHrNa56pk8imvd@tejasproject.ohqojwj.mongodb.net/")

mydb = myclient["mydatabase"]
mycol = mydb["customers"]

def main():
    dblist = myclient.list_database_names()
    if "mydatabase" in dblist:
        print("The database exists.")
    collist = mydb.list_collection_names()
    if "customers" in collist:
        print("The collection exists.")

    mydict = { "name": "John", "address": "Highway 37" }
    x = mycol.insert_one(mydict)
    print(x.inserted_id) 

if __name__ == "__main__":
    main()