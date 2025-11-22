import numpy as np
from matplotlib.offsetbox import OffsetImage
from scipy.ndimage import rotate
import matplotlib.pyplot as plt


def retrieve_image_from_path(image_path: str) -> np.ndarray:
    return np.rot90(plt.imread(image_path, format="png"), 3)


def normalize_image(image: np.ndarray) -> np.ndarray:
    """
    Normalize image data to ensure valid RGB ranges.
    Floats are clipped to [0, 1], integers to [0, 255].
    """
    if image.dtype == np.float32 or image.dtype == np.float64:
        # Clip float data to [0, 1]
        image = np.clip(image, 0, 1)
    elif image.dtype == np.int32 or image.dtype == np.int64 or np.any(image > 255):
        # Clip integer data to [0, 255] and convert to uint8
        image = np.clip(image, 0, 255).astype(np.uint8)
    return image


def create_image_marker(
    image: np.ndarray, desired_width_px: float = 300, angle: float = 0, alpha: float = 1
) -> OffsetImage:
    rotated_image = normalize_image(rotate(image, np.degrees(angle), reshape=True))
    # plt.imshow(rotated_image)
    return OffsetImage(
        rotated_image,
        zoom=desired_width_px / image.shape[0],
        alpha=alpha,
    )
