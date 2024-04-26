package org.carlmontrobotics;

import java.util.HashMap;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;

public class BuildInfoTest {
    @Test
    void testGetInstance() {
        assertNotNull(BuildInfo.getInstance());
    }

    @Test
    void testInitSendable() throws Exception {
        var publishedStrings = new HashMap<String, String>();
        try (SendableBuilder testBuilder = new SendableBuilderImpl() {
            @Override
            public void publishConstString(String key, String value) {
                publishedStrings.put(key, value);
            }
        }) {
            BuildInfo.getInstance().initSendable(testBuilder);
            String actualBuildDate = publishedStrings.get("build_date");
            assertNotNull(actualBuildDate);
            assertTrue(actualBuildDate.matches("[^\"']+"),
                    "build_date (%s) must not contain quotes".formatted(actualBuildDate));
            String actualSha = publishedStrings.get("sha");
            assertNotNull(actualSha);
            assertTrue(actualSha.matches("(UNKNOWN|[a-f0-9]+)"), "sha (%s) is not valid".formatted(actualSha));
        }
    }
}
