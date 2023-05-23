import heapq

class ValueIDManager:
    """
    A class that manages a set of values and their corresponding IDs.
    The IDs are integers that are assigned to values in the order they are added.
    
    The class also keeps track of IDs that have been removed, and reuses them when new values are added.
    
    args: None
    
    methods: 
        remove_by_value(value): Removes a value from the database.
        remove_by_id(id_): Removes a value from the database.
        add_value(value): Adds a value to the database.
        get_id(value): Returns the ID of a value.
        get_value(id_): Returns the value of an ID.
        sync_array(arr): Adds all values in an array to the database, and removes all values that are not in the array.
    
    """
    def __init__(self):
        self.id_to_value = {}
        self.value_to_id = {}
        self.available_ids = []
        self.index_to_entry = {}
        self.next_id = 0


    def remove_by_id(self, id_):
        if id_ in self.id_to_value:
            del_value = self.id_to_value[id_]
            heapq.heappush(self.available_ids, id_)
            del self.id_to_value[id_]
            del self.value_to_id[del_value]
            print(f"ID: {id_} removed.")
        else:
            print(f"No such ID: {id_} in the database.")
            
    def add_value(self, value):
        if value in self.value_to_id:
            print(f"Value: {value} already exists.")
        else:
            if self.available_ids:
                new_id = heapq.heappop(self.available_ids)
            else:
                new_id = self.next_id
                self.next_id += 1

            self.id_to_value[new_id] = value
            self.value_to_id[value] = new_id
            self.index_to_entry[len(self.index_to_entry)] = (value, new_id)  # update new dict
            print(f"Value: {value} added with ID: {new_id}.")

    def remove_by_value(self, value):
        if value in self.value_to_id:
            del_id = self.value_to_id[value]
            del_index = list(self.id_to_value.keys()).index(del_id)
            heapq.heappush(self.available_ids, del_id)
            del self.value_to_id[value]
            del self.id_to_value[del_id]
            #print(self.index_to_entry)
            #del self.index_to_entry[del_index]  # remove from new dict
            for index in range(del_index, len(self.index_to_entry) - 1):
                self.index_to_entry[index] = self.index_to_entry[index + 1]
            # Delete last entry after shifting
            del self.index_to_entry[len(self.index_to_entry) - 1]
            print(f"Value: {value} removed.")
            return del_index
        else:
            print(f"No such value: {value} in the database.")
            return None

    def get_id(self, value):
        return self.value_to_id.get(value, "No such value in the database.")

    def get_value(self, id_):
        return self.id_to_value.get(id_, "No such ID in the database.")

    def sync_array(self, arr):
        removed_ids = []
        for value in arr:
            if value not in self.value_to_id:
                self.add_value(value)
        for value in list(self.value_to_id.keys()): # create a copy of keys to avoid runtime error
            if value not in arr:
                removed_id = self.remove_by_value(value)
                if removed_id is not None:
                    removed_ids.append(removed_id)
        return removed_ids


# Initialize manager
value_id_manager = ValueIDManager()

# Add value
value_id_manager.add_value(3.3)

# Add another value
value_id_manager.add_value(5.5)

# Sync with new array
value_id_manager.sync_array([3.3, 5.5, 7.7, 0.7, 0.0, 7.0, 7.3, 0.3])

# Check state after sync
print(value_id_manager.id_to_value)
print(value_id_manager.value_to_id)

value_id_manager.remove_by_value(7.0)
print(value_id_manager.id_to_value)
print(value_id_manager.value_to_id)
print(value_id_manager.available_ids)
value_id_manager.remove_by_id(1)
print(value_id_manager.available_ids)
value_id_manager.add_value(11.0)
print(value_id_manager.available_ids)
print(value_id_manager.id_to_value)
print(value_id_manager.value_to_id)
# Remove extras not in new array
print(value_id_manager.index_to_entry)
print(value_id_manager.sync_array([3.3, 5.5, 7.7, 0.7, 0.0, 7.0, 7.3,11.0]))

print(value_id_manager.index_to_entry)

