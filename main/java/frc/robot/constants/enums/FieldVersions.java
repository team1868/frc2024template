package frc.robot.constants.enums;

import edu.wpi.first.apriltag.AprilTag;
import java.util.List;
import java.util.Vector;

public enum FieldVersions {
  THEORETICAL_FIELD(
      FieldDims.THEORETICAL_FIELD_DIMS,
      new TagPositions[] {
          TagPositions.OFFICIAL_16H5_TAG_1,
          TagPositions.OFFICIAL_16H5_TAG_2,
          TagPositions.OFFICIAL_16H5_TAG_3,
          TagPositions.OFFICIAL_16H5_TAG_4,
          TagPositions.OFFICIAL_16H5_TAG_5,
          TagPositions.OFFICIAL_16H5_TAG_6,
          TagPositions.OFFICIAL_16H5_TAG_7,
          TagPositions.OFFICIAL_16H5_TAG_8},
      LocationOffsets.THEORETICAL_OFFSETS
  ),
  NASA_FIELD(
      FieldDims.NASA_FIELD_DIMS,
      new TagPositions[] {
          TagPositions.NASA_16H5_TAG_1,
          TagPositions.NASA_16H5_TAG_2,
          TagPositions.NASA_16H5_TAG_3,
          TagPositions.NASA_16H5_TAG_4,
          TagPositions.NASA_16H5_TAG_5,
          TagPositions.NASA_16H5_TAG_6,
          TagPositions.NASA_16H5_TAG_7,
          TagPositions.NASA_16H5_TAG_8},
      LocationOffsets.NASA_OFFSETS
  ),
  COLORADO_FIELD(
      FieldDims.ARIZONA_FIELD_DIMS,
      new TagPositions[] {
          TagPositions.ARIZONA_16H5_TAG_1,
          TagPositions.ARIZONA_16H5_TAG_2,
          TagPositions.ARIZONA_16H5_TAG_3,
          TagPositions.ARIZONA_16H5_TAG_4,
          TagPositions.ARIZONA_16H5_TAG_5,
          TagPositions.ARIZONA_16H5_TAG_6,
          TagPositions.ARIZONA_16H5_TAG_7,
          TagPositions.ARIZONA_16H5_TAG_8},
      LocationOffsets.ARIZONA_OFFSETS
  ),
  SVR_FIELD(
      FieldDims.SVR_FIELD_DIMS,
      new TagPositions[] {
          TagPositions.SVR_16H5_TAG_1,
          TagPositions.SVR_16H5_TAG_2,
          TagPositions.SVR_16H5_TAG_3,
          TagPositions.SVR_16H5_TAG_4,
          TagPositions.SVR_16H5_TAG_5,
          TagPositions.SVR_16H5_TAG_6,
          TagPositions.SVR_16H5_TAG_7,
          TagPositions.SVR_16H5_TAG_8},
      LocationOffsets.SVR_OFFSETS
  );

  public final FieldDims dims;
  public final LocationOffsets offsets;

  public final int numTags;
  public final List<AprilTag> aprilTags;

  FieldVersions(FieldDims fieldDims, TagPositions[] tagArr, LocationOffsets offsets) {
    this.dims = fieldDims == null ? FieldDims.THEORETICAL_FIELD_DIMS : fieldDims;
    this.offsets = offsets == null ? LocationOffsets.THEORETICAL_OFFSETS : offsets;

    tagArr = tagArr == null ? new TagPositions[0] : tagArr;
    numTags = tagArr.length;
    aprilTags = new Vector<AprilTag>();
    for (int i = 0; i < numTags; i++) {
      aprilTags.add(tagArr[i].aprilTag);
    }
  }
}
