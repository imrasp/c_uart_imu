#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import matplotlib.pyplot as plt
import pandas as pd
from datetime import datetime


def get_timestamp(df):
    diff_list, t1, t0, h1, h0 = [], [], [], [], []
    for idx, time in enumerate(df['timestamp']):
        if idx < len(df['timestamp']) - 1:
            diff = df['timestamp'][idx + 1] - df['timestamp'][idx]
            if diff > 10*1e9:
                s1 = epoch_to_human(df['timestamp'][idx + 1])
                s0 = epoch_to_human(df['timestamp'][idx])
                print('t1: {}, {} t0: {}, {} diff: {}'.format(
                    s1,
                    s0,
                    df['timestamp'][idx + 1],
                    df['timestamp'][idx],
                    (df['timestamp'][idx + 1] - df['timestamp'][idx]) / 1e9)
                )
                continue
            else:
                t1.append(df['timestamp'][idx + 1])
                t0.append(df['timestamp'][idx])
                h1.append(epoch_to_human(df['timestamp'][idx + 1]))
                h0.append(epoch_to_human(df['timestamp'][idx]))
                diff_list.append(diff / 1e9)
    return diff_list, t1, t0, h1, h0


def epoch_to_human(epoch):
    dt1 = datetime.fromtimestamp(epoch // 1000000000)
    s1 = dt1.strftime('%Y-%m-%d %H:%M:%S')
    s1 += '.' + str(int(epoch % 1000000000)).zfill(9)
    return s1


def get_first_timestamp(df):
    last_epoch = df['timestamp'].iloc[0]
    print('First timestamp: {} : {}'.format(
        last_epoch,
        epoch_to_human(last_epoch))
    )


def get_last_timestamp(df):
    first_epoch = df['timestamp'].iloc[-1]
    print('Last timestamp: {} : {}'.format(
        first_epoch,
        epoch_to_human(first_epoch))
    )


def plot_hist(df):
    df['time_diff'].plot(kind='hist', bins=10, normed=True)
    plt.show()


def list_error(df, col, factor):
    error = pd.concat([df[df[col] > (df[col].mean() + factor*df[col].std())], df[df[col] < (df[col].mean() - factor*df[col].std())]])
    return error.sort_values([col], ascending=True)

def list_lt_mean(df):
    gt_mean = df[df['time_diff'] < df['time_diff'].mean()]
    return lt_mean.sort_values(['time_diff'], ascending=True)


def get_max_diff(df, time_diff):
    t1 = time_diff['t1'].iloc[0]
    t0 = time_diff['t0'].iloc[0]
    print(df.loc[(df['timestamp'] == t1) | (df['timestamp'] == t0)])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '-f',
        '--filename',
        nargs='+',
        help='CSV file',
        required=True
    )
    args = parser.parse_args()
    pd.set_option('display.width', 180)

    for csv in args.filename:
        print('\n--------------------------------')
        print('------------ {} -------------'.format(csv))
        print('--------------------------------\n')
        df = pd.read_csv(csv)
        get_first_timestamp(df)
        get_last_timestamp(df)
        print('minimum timestamp : {}'.format(df['timestamp'].min()))
        print('maximum timestamp : {}'.format(df['timestamp'].max()))
        print('\n--------------------------------\n')
        diff_list, t1, t0, h1, h0 = get_timestamp(df)
        d = {'time_diff': diff_list, 'h1': h1, 't1': t1, 'h0': h0, 't0': t0}
        time_diff = pd.DataFrame(data=d)
        meand = time_diff['time_diff'].mean()
        mind = time_diff['time_diff'].min()
        maxd = time_diff['time_diff'].max()
        std = time_diff['time_diff'].std()
        print('Min: {}'.format(mind))
        print('Max: {}'.format(maxd))
        print('Mean: {}'.format(meand))
        print('Std: {}'.format(std))
        print('Frequency: {} Hz'.format(1 / meand))
        
        error_list = list_error(
            time_diff[['t1', 't0', 'h1', 'h0', 'time_diff']],
            'time_diff', 2
        )
        
        print('\n--------------------------------\n')
        print('# of error: {}'.format(error_list['time_diff'].count()))
        
        print('\nDiff greater than mean')
        print(error_list.tail(n=5))
        print('\n--------------------------------\n')
        print('\nDiff less than mean')
        print(error_list.head(n=5))
        print('\n--------------------------------\n')
        if csv.endswith('imu0.csv'):
            print('mean xgyro : {}'.format(df['omega_x'].mean()))
            print('mean ygyro : {}'.format(df['omega_y'].mean()))
            print('mean zgyro : {}'.format(df['omega_z'].mean()))
            print('mean xzcc : {}'.format(df['alpha_x'].mean()))
            print('mean yzcc : {}'.format(df['alpha_y'].mean()))
            print('mean zzcc : {}'.format(df['alpha_z'].mean()))
            acc_gyro = ['omega_x', 'omega_y', 'omega_z', 'alpha_x', 'alpha_y', 'alpha_x']
            for each in acc_gyro:
                error_list = list_error(df, each, 2)
                print('#error {} : {}'.format(each, error_list[each].count()))
                print(error_list.head(n=3))
                print('...')
                print(error_list.tail(n=3))
                print('\n--------------------------------\n')
            # plot_hist(gt_mean)
            # df.plot(x='timestamp')
            # plt.show()
