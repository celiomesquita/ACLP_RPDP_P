
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd



class TTT(object):
    def __init__(self, name_list, path_list):
        self.name_list = name_list
        self.path_list = path_list

    def plot_series(self, df, name):
        n = len(df) # df -> dataframe

        p = np.array([(i + 0.5) / (n + 1) for i in range(n)])

        # finding the PDF of the histogram using count values
        pdf = p / sum(p)
        
        # using numpy np.cumsum to calculate the CDF
        # We can also find using the PDF values by looping and adding
        cdf = np.cumsum(pdf)

        # plotting PDF and CDF

        # using numpy array
        # plt.plot(df[:,0], cdf, label=f"cdf: {name} | {n} solutions")
        # plt.scatter(df[:,0], cdf)
        # avg = np.mean(df, axis=0) # 0 = vertical axis

        # using pandas dataframe
        plt.plot(df.iloc[:, 0], cdf, label=f"cdf: {name} | {n} solutions")
        plt.scatter(df.iloc[:, 0], cdf)
        avg = df.iloc[:, 0].mean()

        plt.axvline(x=avg, color='pink', linestyle='-')
        # plt.axhline(y=0.8, color='pink', linestyle='-')


    # time to target plot
    def Plot(self):

        # Configure Plot
        plt.figure(figsize=(15, 12))

        plt.style.use('seaborn')

        plt.rcParams['font.size'] = 11

        # Plot Text
        plt.title("Time-to-target Plot")

        plt.xlabel("Time (sec)")
        plt.ylabel("Cumulative Probability")

        plt.ylim(0.0, 1.1)


        # df = pd.read_csv("C:/Users/kennethcassel/homes.csv")
        # sorted_df = df.sort_values(by=["price"], ascending=False)        

        for name, path in zip(self.name_list, self.path_list):

            df = pd.read_csv(path, header=None, names=['time', 'value'])
            df = df.sort_values(df.columns[0])

            # --- another way to read and sort a file
            # matrix = []
            # with open(path, "r") as file:
            #     lines = file.readlines()
            #     for i, line in enumerate(lines):
            #         cols = line.split(",")
            #         x = float(cols[0])
            #         y = float(cols[1])
            #         matrix.append([x,y])
            # t = t[t[:, 0].argsort()] # sort matrix by the first column

            # t = np.array(t)

            self.plot_series(df, name)
       

        # Add plot legend
        plt.legend()

        plt.show()
