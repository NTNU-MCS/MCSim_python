import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

class DataVisualizer:    
    def __init__ (self, data_logger, plot_info):
        self._data_logger = data_logger
        self._plot_info = plot_info 

    def plot_data(self, atr, x, y):
        plt.ion()
        x = self._data_logger.sorted_data[atr][x]
        print(x)
        y = self._data_logger.sorted_data[atr][y]
        print(y)
        plt.gca().cla() # optionally clear axes
        plt.plot(x, y)
        plt.title(atr)  
        return plt
    
    def stop(self):
        self._running = False
    
    # def start(self):
    #     self._running = True
    #     print('DataVisualizer running.')
    #     while self._running:
    #         self._plot_data()
        
    #     print('DataVisualizer stopped.')
    