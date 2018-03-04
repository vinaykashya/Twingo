from styx_msgs.msg import TrafficLight

import tensorflow as tf
import os
import cv2
import numpy as np
import rospy

class TLClassifier(object):
    def __init__(self, model_dir = None):
        self.traffic_color_dict = {0: TrafficLight.RED, 1: TrafficLight.GREEN, 2: TrafficLight.YELLOW}
        detection_model_path = 'tf_graphs/ssd_mobilenet_v1_coco_2017_11_17/frozen_inference_graph.pb'
        if not os.path.exists(detection_model_path):
            rospy.logerr('Detection model not found at {}'.format(detection_model_path))
        classification_model_path = 'tf_graphs/classification/SqueezenetV1.pb'
        if not os.path.exists(classification_model_path):
            rospy.logerr('Classification model not found at {}'.format(classification_model_path))

        self.config = tf.ConfigProto()
        self.config.gpu_options.allow_growth = True
        self.config.gpu_options.per_process_gpu_memory_fraction = 1
        jit_level = tf.OptimizerOptions.ON_1
        self.config.graph_options.optimizer_options.global_jit_level = jit_level


        self.graph_detection = self.graph_load(detection_model_path, self.config)
	self.detection_session_tf = tf.Session(graph=self.graph_detection, config=self.config)
	self.image_tensor = self.graph_detection.get_tensor_by_name('image_tensor:0')
        self.detection_boxes = self.graph_detection.get_tensor_by_name('detection_boxes:0')
        self.detection_classes = self.graph_detection.get_tensor_by_name('detection_classes:0')
	self.detection_scores = self.graph_detection.get_tensor_by_name('detection_scores:0')

        self.graph_classification = self.graph_load(classification_model_path, self.config)
        self.classification_session_tf = tf.Session(graph=self.graph_classification, config=self.config)
        self.in_graph = self.graph_classification.get_tensor_by_name('input_1_1:0')
        self.out_graph = self.graph_classification.get_tensor_by_name('output_0:0')


        ## preload model weights
        self.detection(cv2.cvtColor(np.zeros((600, 800), np.uint8), cv2.COLOR_GRAY2RGB))
        self.classification(cv2.cvtColor(np.zeros((32, 32), np.uint8), cv2.COLOR_GRAY2RGB))

    def get_classification(self, image):
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
	    self.im_height, self.im_width, _ = image.shape
            box = self.detection(image)
            if box == None:
                return TrafficLight.UNKNOWN
            left, right, top, bottom = box
            img_crop = image[top:bottom, left:right]
            traffic_light = cv2.resize(img_crop, (32, 32))
            return self.classification(traffic_light)

    def detection(self, image):
        image_expanded = np.expand_dims(image, axis=0)
        with self.detection_session_tf.as_default(), self.graph_detection.as_default(): 
            bounding_boxes, detection_confidence_scores, classes = self.detection_session_tf.run(
                [self.detection_boxes, self.detection_scores, self.detection_classes],
                feed_dict={self.image_tensor: image_expanded})
            return self.non_maximal_supprssion(bounding_boxes, detection_confidence_scores, classes)

    def non_maximal_supprssion(self,boxes, confidence_scores, classes):
	    confidence = 0.1
	    max_conf = 0
	    traffic_class = 10
	    box_id = -1
	    boxes = np.squeeze(boxes)
	    classes = np.squeeze(classes).astype(np.int32)
	    confidence_scores = np.squeeze(confidence_scores)

	    for i in range(boxes.shape[0]):
		if classes[i] == traffic_class and confidence_scores[i] > confidence:
		    if confidence_scores[i] > max_conf:
		        max_conf = confidence_scores[i]
		        box_id = i
	    if box_id != -1:
		box = tuple(boxes[box_id].tolist())
		ymin, xmin, ymax, xmax = box
		(left, right, top, bottom) = (xmin * self.im_width -5 if xmin * self.im_width -5 > 0 else 0, xmax * self.im_width +5 if xmax * self.im_width +5 < self.im_width else self.im_width, ymin * self.im_height-10 if ymin * self.im_height-10 > 0 else 0, ymax * self.im_height+10 if ymax * self.im_height+10 < self.im_height else self.im_height)
		box = int(left), int(right), int(top), int(bottom)
		return box
	    else:
		return None


    def classification(self, image):
        with self.classification_session_tf.as_default(), self.graph_classification.as_default():
            detection_output = list(self.classification_session_tf.run(tf.nn.softmax(self.out_graph.eval(feed_dict={self.in_graph: [image]}))))
	    max_conf_light_color = max(detection_output)
            traffic_light_ind = detection_output.index(max_conf_light_color)
        return self.traffic_color_dict[traffic_light_ind]


    def graph_load(self,graph_file, config):
	    with tf.Session(graph=tf.Graph(), config=config) as sess:
		assert tf.get_default_session() is sess
		graphdefinition = tf.GraphDef()
		with tf.gfile.Open(graph_file, 'rb') as f:
		    data = f.read()
		    graphdefinition.ParseFromString(data)
		tf.import_graph_def(graphdefinition, name='')
		graph = tf.get_default_graph()
		return graph

