#!/usr/bin/env python3
"""
Mock YOLO model for testing when actual model is not available
"""
import numpy as np

class MockYOLO:
    def __init__(self, model_path=""):
        self.model_path = model_path
        print(f"[MOCK] Loading YOLO model from {model_path}")
        
    def detect(self, image):
        """
        Mock detection - returns empty detections
        """
        return [], [], []  # boxes, scores, class_ids
        
    def __call__(self, image):
        return self.detect(image)