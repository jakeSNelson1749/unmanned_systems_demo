import pandas as pd
import numpy as np

def add(x:float,y:float) -> float:
    return x+y



dataset = pd.read_csv("/home/kingofclubs/coursework/Unmanned_Systems/unmanned_systems_demo/toy_data/toy_data_complementary_filter.csv")

print("dataset is", dataset)
time_data = dataset["t_s"]