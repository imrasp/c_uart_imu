import pymap3d as pm
import pandas as pd

def convert_ned2enu(filename):
    df = pd.read_csv(filename, float_precision='round_trip')
    df_conversion_result = df
    for idx, time in enumerate(df['timestamp']):
        if idx < len(df['timestamp']) - 1:
            #Gyro
            df_conversion_result['alpha_z'] = df['alpha_z'] * -1    # U = -D
            df_conversion_result['alpha_y'] = df['alpha_y'] * -1    # N = -E
            df_conversion_result['alpha_x'] = df['alpha_x']         # E = N
            #Accelero
            df_conversion_result['omega_z'] = df['omega_z'] * -1    # U = -D
            df_conversion_result['omega_y'] = df['omega_y'] * -1    # N = -E
            df_conversion_result['omega_x'] = df['omega_x']         # E = N

    df_conversion_result.to_csv('imu0_enu.csv', index=False)

if __name__ == '__main__':
    #FN
    convert_ned2enu('imu0_gps.csv')
    print ("Finish")