# Training Data Capture

How we capture images for training the object detector.

## Camera

- Use the same webcam that is mounted on the sensor rig, connected to a laptop.
- Record video at **640×480** (same as the robot-side camera topic).
- Turn off video stabilization and HDR in the camera app. They crop or
  change the image, so frames would differ from what the robot sees.
- Keep the camera height and tilt close to the sensor rig setup.

## Recording

- Objects: box (lying), box (standing), translucent plastic cup, power strip.
- About 200 images per object, plus about 100 images of empty floor
  (no objects, empty label files).
- Distance: vary between about 1 m and 2.5 m.
- Position in the image: mix left, center and right, including the sides
  of the field of view.
- Vary background and lighting across sessions.
- Keep moving the object or the camera while recording, so that frames
  are not near-duplicates.
- Scenes recorded at the evaluation site must come from sessions that are
  not used for evaluation.

## Frame extraction

- Extract frames at a fixed **time** interval (e.g. every 0.5 s), not every
  N frames. Camera apps often record with a variable frame rate.
- Check that the extracted images are 640×480 before labeling.

## Data handling

- Videos and images are **not** committed to this repository.
