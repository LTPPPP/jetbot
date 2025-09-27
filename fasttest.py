#!/usr/bin/env python3
"""
Fast test với loading optimization và progress indicator
"""

import os
import sys
import time
import threading
import numpy as np

def show_loading_progress():
    """Hiển thị progress loading"""
    chars = "⠋⠙⠹⠸⠼⠴⠦⠧⠇⠏"
    i = 0
    while loading:
        sys.stdout.write(f"\r🔄 Loading model... {chars[i % len(chars)]}")
        sys.stdout.flush()
        time.sleep(0.1)
        i += 1

def main():
    global loading
    print("🚀 Fast ObjectDetector Test")
    print("=" * 30)
    
    # 1. Kiểm tra file
    model_path = "./ssd_mobilenet_v2_coco.engine"
    if not os.path.exists(model_path):
        print(f"❌ Model không tồn tại: {model_path}")
        return
    
    file_size = os.path.getsize(model_path)
    print(f"📁 Model: {model_path} ({file_size/1024/1024:.1f} MB)")
    
    # 2. Import với progress
    print("🔄 Importing libraries...")
    try:
        from jetbot.object_detection import ObjectDetector
        print("✅ Import OK")
    except Exception as e:
        print(f"❌ Import error: {e}")
        return
    
    # 3. Load model với progress indicator
    print("\n💡 TensorRT engine loading (có thể mất 30-60s trên Jetson Nano)...")
    print("   - Lần đầu load sẽ lâu nhất")
    print("   - TensorRT đang optimize engine cho GPU")
    print("   - Các lần sau sẽ nhanh hơn")
    
    loading = True
    progress_thread = threading.Thread(target=show_loading_progress)
    progress_thread.daemon = True
    progress_thread.start()
    
    try:
        start_time = time.time()
        detector = ObjectDetector(model_path)
        load_time = time.time() - start_time
        loading = False
        
        print(f"\r✅ Model loaded! ({load_time:.1f}s)                    ")
        
    except Exception as e:
        loading = False
        print(f"\r❌ Load error: {e}                    ")
        return
    
    # 4. Quick test
    print("🔄 Quick inference test...")
    dummy_image = np.random.randint(0, 255, (300, 300, 3), dtype=np.uint8)
    
    start_time = time.time()
    detections = detector(dummy_image)
    inference_time = time.time() - start_time
    
    print(f"✅ Inference OK! ({inference_time*1000:.0f}ms)")
    print(f"📊 Objects: {len(detections) if detections else 0}")
    
    # 5. Performance tips
    print(f"\n💡 Performance Tips:")
    print(f"   - Load time: {load_time:.1f}s (normal cho Jetson Nano)")
    print(f"   - Inference: {inference_time*1000:.0f}ms")
    print(f"   - Expected FPS: ~{1/inference_time:.0f}")
    print(f"   - Model sẽ load nhanh hơn lần sau")
    
    print(f"\n🎉 Test hoàn thành! Model sẵn sàng sử dụng.")

if __name__ == "__main__":
    loading = False
    main()