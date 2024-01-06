package frc.robot.utils;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.DriverStation;
import java.io.File;
import java.io.IOException;

public class FileUtils {
  public static void checkForFile(File file) throws IOException {
    if (!file.exists()) {
      DriverStation.reportError("File: " + file.getPath() + " " + file + " not found!", true);
      throw new IOException("No such file: " + file);
    }
  }

  /**
   * Open JSON file.
   *
   * @param file JSON File to open.
   * @return JsonNode of file.
   */
  private static JsonNode openJson(File file) {
    try {
      return new ObjectMapper().readTree(file);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }
}
