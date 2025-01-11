def stringReplaceIndex(string,index,new_char) -> str:
    string_list = list(string)
    string_list[index] = new_char
    new_string = "".join(string_list)
    
    return new_string