#! /usr/bin/python3
import pickle
import numpy as np
# import tensorflow as tf
# from tensorflow import keras...  みたいな感じ


class MyROSModel:
    # see https://www.tensorflow.org/tutorials/keras/regression?hl=ja#%E3%83%A2%E3%83%87%E3%83%AB
    def __init__(self, saved_path=None):
        model = tf.keras.Sequential([
            layers.Dense(units=64),
            layers.Dense(units=64),
            ....
        ])
        model.compile(
            optimizer=tf.optimizers.Adam(learning_rate=0.1),
            loss='mean_absolute_error')
        self.model = model
        rospy.init_node('model_server')
       s = rospy.Service('dataset_save', SrvCls, self.save_data)
       s = rospy.Service('model_fit', SrvCls, self.model_fit)
       s = rospy.Service('model_predict', SrvCls, self.model_predict)
       data = pickle.load(open('/home/amabe/data.pkle'))

       # 参考
       # import pickle
       # data = {'s': list(range(10))}
       # pickle.dump(data , open('/tmp/my_data.pickle', 'wb'))
       # read_data = pickle.load( open('/tmp/my_data.pickle', 'rb'))
       # read_data

    def model_fit(self, req):
        data = pickle.load(open('/home/amabe/data.pkle'))
        status = np.array(data['status'])
        ...
        self.model.fit(train, val, epoch = 100)
        self._save_model()

    def model_predict(self, req):
        s = np.array(req.status)
        action = self.model.predicct(s)
        res = RoboAction(action=action)
        return res

    def _save_model_param(self):
        self.model.save('/tmp/hoge')

    def save_data(self, req):
        s = np.array(req.status)
        action_del = np.array(req.action_delta)
        data = pickle.load(open('/home/amabe/data.pkle', 'rb'))
        data['status'].append(s)
        data['action_delta'].append(action_del)
        pickle.dump(open('/home/amabe/data.pkle', 'wb'), data)
        return res
