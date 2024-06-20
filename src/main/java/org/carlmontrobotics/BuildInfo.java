package org.carlmontrobotics;

import java.util.Properties;
import java.io.File;
import java.io.InputStream;
import java.nio.file.Path;
import java.nio.file.Files;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Filesystem;

public class BuildInfo implements Sendable {
    private Properties props = new Properties();

    static private BuildInfo instance = null;

    public static BuildInfo getInstance() {
        if (instance == null) {
            instance = new BuildInfo();
        }
        return instance;
    }

    private BuildInfo() {
        Path path = Path
                .of(Filesystem.getDeployDirectory().getAbsolutePath() + File.separator + "BuildInfo.properties");
        try (InputStream is = Files.newInputStream(path)) {
            props.load(is);
        } catch (Exception ex) {
            System.err.println("Error reading build properties from %s".formatted(path));
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        props.stringPropertyNames().forEach(name -> {
            var value = props.getProperty(name);
            // Workaround bug (https://github.com/lessthanoptimal/gversion-plugin/pull/14)
            // where the gversion plugin surrounds values with quotes.
            value = value.replaceAll("\"", "");
            builder.publishConstString(name, value);
        });
    }
}
