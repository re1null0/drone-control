# from flask import Flask, Response
# import cv2
# from ultralytics import YOLO 

# app = Flask(__name__)
# camera = cv2.VideoCapture(0)
# # model = YOLO("yolov8s.pt")  


# def generate_frames():
#     while True:
#         success, frame = camera.read()
#         if not success:
#             print("Failed to read frame")
#             break

#         _, buffer = cv2.imencode('.jpg', frame)
#         frame_bytes = buffer.tobytes()

#         yield (b'--frame\r\n'
#                b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        

# # def generate_frames():
# #     while True:
# #         success, frame = camera.read()
# #         if not success:
# #             break
        
# #         # Add YOLOv8 inference
# #         results = model(frame, conf=0.5)  # Adjust confidence threshold
# #         annotated_frame = results[0].plot()  # Draw detections on frame
        
# #         # Use annotated frame instead of original
# #         _, buffer = cv2.imencode('.jpg', annotated_frame)
# #         frame_bytes = buffer.tobytes()
        
# #         yield (b'--frame\r\n'
# #                b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

# @app.route('/video_feed')
# def video_feed():
#     return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

# @app.after_request
# def cors(response):
#     response.headers['Access-Control-Allow-Origin'] = '*'
#     return response

# if __name__ == '__main__':
#     app.run(host='0.0.0.0', port=5001)
from flask import Flask, Response, render_template_string
import cv2
import torch
import os
import time
from datetime import datetime
from PIL import Image
from ultralytics import YOLO
from transformers import AutoProcessor, AutoModelForImageTextToText
from torchvision.transforms import functional as F, InterpolationMode
import threading
import queue

app = Flask(__name__)
camera = cv2.VideoCapture(0)
model_yolo = YOLO("best.pt")  # Load YOLOv8 model

# Create a directory to store snapshots
SNAPSHOT_DIR = "fire_snapshots"
os.makedirs(SNAPSHOT_DIR, exist_ok=True)

# Queue for communication between detection and analysis threads
analysis_queue = queue.Queue()
results_queue = queue.Queue()

# Global variables to store the latest analysis
latest_analysis = "No fire detected yet"
last_detection_time = 0
DETECTION_COOLDOWN = 10  # Seconds between detections to avoid too many analyses

# Set up Qwen2-Wildfire-2B model
hf_token = "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"

# Initialize the model and processor in a separate function to be called in a thread
def initialize_wildfire_model():
    global processor, model
    
    # Load processor with correct size parameters
    processor = AutoProcessor.from_pretrained(
        "hiko1999/Qwen2-Wildfire-2B",
        token=hf_token,
        use_fast=True,
        do_resize=True,
        size={"shortest_edge": 448, "longest_edge": 448}
    )

    # Load the model
    model = AutoModelForImageTextToText.from_pretrained(
        "hiko1999/Qwen2-Wildfire-2B",
        token=hf_token
    )

    # Move model to CPU
    model.to("cpu")
    print("Wildfire analysis model loaded successfully")

# Custom preprocessing function
def custom_process_vision_info(image):
    """Process a single image for the model."""
    try:
        # Force resize to 448x448 (multiple of 14) to avoid dimension issues
        image = F.resize(image, (448, 448), interpolation=InterpolationMode.BICUBIC)
        print(f"Processed image with dimensions: {image.size}")
        return image
    except Exception as e:
        print(f"Error processing image: {e}")
        raise

# Function to analyze fire image
def analyze_fire_image(image_path):
    try:
        # Open image
        pil_image = Image.open(image_path).convert("RGB")
        
        # Prepare messages with the image
        messages = [
            {
                "role": "user",
                "content": [
                    {"type": "image", "image": image_path},
                    {"type": "text", "text": "Provide a concise analysis of: 1) Fire type and environmental description, 2) Flame characteristics (color, height, intensity), 3) Smoke characteristics, 4) Fire behavior, 5) Affected area, 6) Environmental factors, 7) Recommended containment actions."}
                ]
            }
        ]

        # Create prompt text
        try:
            prompt_text = processor.tokenizer.apply_chat_template(
                messages, tokenize=False, add_generation_prompt=True
            )
        except AttributeError:
            prompt_text = processor.apply_chat_template(
                messages, tokenize=False, add_generation_prompt=True
            )

        # Process the image
        processed_image = custom_process_vision_info(pil_image)
        
        # Tokenize and prepare inputs
        inputs = processor(
            text=[prompt_text],
            images=[processed_image],
            videos=None,
            padding=True,
            return_tensors="pt",
            do_resize=False
        )
        
        # Move inputs to CPU
        inputs = {k: v.to("cpu") for k, v in inputs.items()}
        
        # Generate response
        with torch.no_grad():
            generated_ids = model.generate(**inputs, max_new_tokens=256)
        
        # Extract only the generated part
        generated_ids_trimmed = [
            out_ids[len(in_ids):] for in_ids, out_ids in zip(inputs["input_ids"], generated_ids)
        ]
        
        # Decode the generated text
        try:
            output_text = processor.tokenizer.batch_decode(
                generated_ids_trimmed,
                skip_special_tokens=True,
                clean_up_tokenization_spaces=False
            )[0]
        except AttributeError:
            output_text = processor.batch_decode(
                generated_ids_trimmed,
                skip_special_tokens=True,
                clean_up_tokenization_spaces=False
            )[0]
        
        return output_text
    
    except Exception as e:
        print(f"Error during fire analysis: {e}")
        import traceback
        traceback.print_exc()
        return f"Analysis error: {str(e)}"

# Thread function for analyzing fire images
def analysis_worker():
    while True:
        image_path = analysis_queue.get()
        if image_path is None:
            break
            
        print(f"Analyzing fire image: {image_path}")
        analysis_result = analyze_fire_image(image_path)
        results_queue.put(analysis_result)
        analysis_queue.task_done()

def generate_frames():
    global latest_analysis, last_detection_time
    
    while True:
        success, frame = camera.read()
        if not success:
            print("Failed to read frame")
            break
        
        # Run YOLOv8 inference
        results = model_yolo(frame, conf=0.5)
        
        # Check if "fire" is among the detected classes
        fire_detected = False
        for result in results:
            if hasattr(result, 'names') and result.names:
                for box_idx, box in enumerate(result.boxes):
                    cls_id = int(box.cls[0].item())
                    cls_name = result.names[cls_id]
                    if cls_name.lower() == "fire":
                        fire_detected = True
                        break
        
        # If fire is detected and cooldown period has passed
        current_time = time.time()
        if fire_detected and (current_time - last_detection_time) > DETECTION_COOLDOWN:
            last_detection_time = current_time
            
            # Save snapshot
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            snapshot_path = os.path.join(SNAPSHOT_DIR, f"fire_{timestamp}.jpg")
            cv2.imwrite(snapshot_path, frame)
            print(f"Fire detected! Snapshot saved to {snapshot_path}")
            
            # Queue for analysis
            analysis_queue.put(snapshot_path)
        
        # Check if new analysis results are available
        try:
            while not results_queue.empty():
                latest_analysis = results_queue.get_nowait()
                results_queue.task_done()
        except queue.Empty:
            pass
        
        # Draw YOLOv8 detections
        annotated_frame = results[0].plot()
        
        # Add analysis text to the frame
        font = cv2.FONT_HERSHEY_SIMPLEX
        y_position = 30
        
        # Add a semi-transparent overlay for text background
        overlay = annotated_frame.copy()
        cv2.rectangle(overlay, (10, 10), (800, 200), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, annotated_frame, 0.4, 0, annotated_frame)
        
        # Split analysis into lines and display
        analysis_lines = latest_analysis.split('\n')
        for i, line in enumerate(analysis_lines[:5]):  # Limit to first 5 lines
            cv2.putText(annotated_frame, line, (15, y_position), font, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
            y_position += 30
        
        # Convert to JPEG
        _, buffer = cv2.imencode('.jpg', annotated_frame)
        frame_bytes = buffer.tobytes()
        
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/')
def index():
    # Simple HTML template with video feed and analysis results
    return render_template_string('''
    <!DOCTYPE html>
    <html>
    <head>
        <title>Fire Detection and Analysis</title>
        <style>
            body { font-family: Arial, sans-serif; margin: 20px; background-color: #f5f5f5; }
            .container { display: flex; flex-direction: column; align-items: center; }
            .video-container { margin-bottom: 20px; }
            .analysis-container { 
                width: 80%; 
                max-width: 800px; 
                background-color: white; 
                padding: 20px; 
                border-radius: 8px;
                box-shadow: 0 2px 4px rgba(0,0,0,0.1);
            }
            h1 { color: #d32f2f; }
            h2 { color: #f44336; }
        </style>
    </head>
    <body>
        <div class="container">
            <h1>Real-time Fire Detection and Analysis</h1>
            <div class="video-container">
                <img src="{{ url_for('video_feed') }}" width="800">
            </div>
        </div>
    </body>
    </html>
    ''')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.after_request
def cors(response):
    response.headers['Access-Control-Allow-Origin'] = '*'
    return response

if __name__ == '__main__':
    # Initialize the wildfire model
    print("Loading wildfire analysis model...")
    initialize_wildfire_model()
    
    # Start the analysis worker thread
    analysis_thread = threading.Thread(target=analysis_worker, daemon=True)
    analysis_thread.start()
    
    # Run the Flask app
    app.run(host='0.0.0.0', port=5001)
