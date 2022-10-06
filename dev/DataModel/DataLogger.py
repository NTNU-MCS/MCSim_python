class DataLogger:
    def __init__(self, stream_parser):
        self.__stream_parser = stream_parser
        self.sorted_data = {}
        self.__running = False
        self.__buffer_data = stream_parser.parsed_msg_list
    
    def log_buffered_message(self):
        if len(self.__buffer_data) < 1: return
        
        print(self.__buffer_data[-1])
        self.__stream_parser.pop_parsed_msg_list()

    def sort_buffered_data(self):
        self.__running = True

        while self.__running:
            self.log_buffered_message()
