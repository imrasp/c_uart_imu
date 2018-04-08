# -*- coding: utf-8 -*-
import pandas as pd
from datetime import datetime

def get_timestamp_offset(filename, filename2):
    df = pd.read_csv(filename,float_precision='round_trip')
    df2 = pd.read_csv(filename2)
    df_result = df
    for idx, time in enumerate(df['timestamp']):
        if idx < len(df['timestamp']) - 1:
            df_result['timestamp'][idx+1] = df['timestamp'][0] + ((df2['timestamp'][idx+1] - df2['timestamp'][0]) * 1e3)
            print (epoch_to_human(df_result['timestamp'][idx+1]))

    df_result.to_csv('imu0_result1.csv', index=False)

def epoch_to_human(epoch):
    dt1 = datetime.fromtimestamp(epoch // 1000000000)
    s1 = dt1.strftime('%Y-%m-%d %H:%M:%S')
    s1 += '.' + str(int(epoch % 1000000000)).zfill(9)
    return s1

def mean(numbers):
    return float(sum(numbers)) / max(len(numbers), 1)

if __name__ == '__main__':
    get_timestamp_offset('imu0_odroidunix.csv','imu0_pixhawkms.csv')
    print ("Finish")
