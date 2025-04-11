import torch
from PIL import Image
from transformers import AutoProcessor, AutoModelForImageTextToText
from torchvision.transforms import functional as F, InterpolationMode

def custom_process_vision_info(messages):
    """Extract and properly preprocess images from messages."""
    images = []
    video_paths = []
    
    for message in messages:
        if message.get("role") == "user":
            for content in message.get("content", []):
                if content.get("type") == "image":
                    img_path = content.get("image", "")
                    try:
                        # Load image
                        image = Image.open(img_path).convert("RGB")
                        
                        # Force resize to 448x448 (multiple of 14) to avoid dimension issues
                        image = F.resize(image, (448, 448), interpolation=InterpolationMode.BICUBIC)
                        
                        images.append(image)
                        print(f"Processed image with dimensions: {image.size}")
                    except Exception as e:
                        print(f"Error loading image {img_path}: {e}")
                        raise
                elif content.get("type") == "video":
                    video_paths.append(content.get("video"))
    return images, video_paths

# Set your Hugging Face token
hf_token = "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"

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

# Prepare messages with an image and a text prompt
image_path = "/Users/shyryn/Desktop/Penn/YOLOv8-Fire-and-Smoke-Detection/datasets/fire-8/train/images/fire2_mp4-75_jpg.rf.34a416c8e0086090e9226d8a457b8cdf.jpg"
messages = [
    {
        "role": "user",
        "content": [
            {"type": "image", "image": image_path},
            {"type": "text", "text": "comment on Fire type and environmental description, Flame characteristics (color, height, intensity), Smoke characteristics (color, density, spread direction), Fire behavior (spread speed, movement pattern), Affected area description (fire coverage, vegetation or structures involved), Environmental factors (surrounding terrain, wind speed, temperature)."}
        ]
    }
]

# Create prompt text using the processor's chat template
try:
    prompt_text = processor.tokenizer.apply_chat_template(
        messages, tokenize=False, add_generation_prompt=True
    )
except AttributeError:
    # Fallback if processor doesn't have tokenizer attribute directly
    prompt_text = processor.apply_chat_template(
        messages, tokenize=False, add_generation_prompt=True
    )

# Use custom preprocessing function
image_inputs, video_inputs = custom_process_vision_info(messages)

# Tokenize and prepare inputs
try:
    inputs = processor(
        text=[prompt_text],
        images=image_inputs,
        videos=None,  # Pass None instead of empty list
        padding=True,
        return_tensors="pt",
        do_resize=False  # Disable internal resizing
    )
    
    # Move inputs to CPU
    inputs = {k: v.to("cpu") for k, v in inputs.items()}
    
    # Generate response
    with torch.no_grad():
        generated_ids = model.generate(**inputs, max_new_tokens=128)
    
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
        )
    except AttributeError:
        # Fallback if processor doesn't have tokenizer attribute directly
        output_text = processor.batch_decode(
            generated_ids_trimmed,
            skip_special_tokens=True,
            clean_up_tokenization_spaces=False
        )
    
    print("Inference Output:", output_text)
    
except Exception as e:
    print(f"Error during processing: {str(e)}")
    import traceback
    traceback.print_exc()
