#! /usr/bin/python3
import pickle
import numpy as np

import rospy
from DatasetSave.srv import
from ModelFit.srv import
from ModelPredict.srv import 

import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import  layrers

class MyROSModel:
    # see https://www.tensorflow.org/tutorials/keras/regression?hl=ja#%E3%83%A2%E3%83%87%E3%83%AB
    def __init__(self, saved_path=None):
        model = tf.keras.Sequential([
            layers.Dense(units=64, activation='relu', input_shape=[]),
            layers.Dense(units=64) activation='relu',
            layers.Dense(2) #XY
        ])
        model.compile(
            optimizer=tf.optimizers.Adam(learning_rate=0.1),
            loss='mean_absolute_error')
        self.model = model
        rospy.init_node('model_server')
        s = rospy.Service('dataset_save', サービス型, self.save_data)
        s = rospy.Service('model_fit', SrvCls, self.model_fit)
        s = rospy.Service('model_predict', SrvCls, self.model_predict)
        data = pickle.load(open('/home/amabe/franka_ws/src/jsk_demos/jsk_2021_fix_kxr/action_delta/data.pickle'))

       # 参考
       # import pickle
       # data = {'s': list(range(10))}
       # pickle.dump(data , open('/tmp/my_data.pickle', 'wb'))
       # read_data = pickle.load( open('/tmp/my_data.pickel', 'rb'))
       # read_data

    def model_fit(self, req):
        data = pickle.load(open('/home/amabe/franka_ws/src/jsk_demos/jsk_2021_fix_kxr/action_delta/data.pickle'))
        status = np.array(data['status'])
        ...
        self.model.fit(train, val, epoch = 100)
        self._save_model()

    def model_predict(self, req):
        s = np.array(req.status)
        action = self.model.predicct(s)
        res = RoboAction(action=action)
        return res

    def save_model_param(self):
        with open('/home/amabe/franka_ws/src/jsk_demos/jsk_2021_fix_kxr/action_delta/data.pickle','wb') as f:
            pickle.dump(self.save.model??, f)
        self.model.save('/home/amabe/franka_ws/src/jsk_demos/jsk_2021_fix_kxr/action_delta/data.pickle')

    def save_data(self, req):
        s = np.array(req.status)
        action_delta = np.array(req.action_delta)
        data = pickle.load(open('/home/amabe/data.pkle', 'rb'))
        data['status'].append(s)
        data['action_delta'].append(action_del)
        pickle.dump(open('/home/amabe/data.pkle', 'wb'), data)
        return res
