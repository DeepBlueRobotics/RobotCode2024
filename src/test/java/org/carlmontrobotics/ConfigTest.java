package org.carlmontrobotics;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.condition.EnabledIfEnvironmentVariable;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.HashMap;

public class ConfigTest {
    @Test
    void testIsExampleFlagEnabled() {
        assertEquals(false, new Config() {
        }.isExampleFlagEnabled());
        assertEquals(true, new Config() {
            {
                this.exampleFlagEnabled = true;
            }
        }.isExampleFlagEnabled());
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
            Config testConfig = new Config() {
            };
            testConfig.initSendable(testBuilder);
            assertEquals(new HashMap<String, String>(), publishedStrings,
                    "A default config should not publish anything.");

            testConfig = new Config() {
                {
                    this.exampleFlagEnabled = true;
                }
            };
            testConfig.initSendable(testBuilder);
            assertEquals(new HashMap<String, String>() {
                {
                    this.put("isExampleFlagEnabled()", "true (default is false)");
                }
            }, publishedStrings, "A config with overrides should publish what is overriden.");
        }
    }

    @Test
    public void testToString() {
        assertEquals("Using default config values", new Config() {
        }.toString());
        assertEquals("WARNING: USING OVERRIDDEN CONFIG VALUES\nisExampleFlagEnabled() returns true (default is false)",
                new Config() {
                    {
                        exampleFlagEnabled = true;
                    }
                }.toString());
    }

    @Test
    @EnabledIfEnvironmentVariable(named = "testCONFIGIsDefault", matches = "true", disabledReason = "not trying to modify GitHub master")
    public void testNoConfigSettingsOverridden() throws Exception {
        var publishedStrings = new HashMap<String, String>();
        try (SendableBuilder testBuilder = new SendableBuilderImpl() {
            @Override
            public void publishConstString(String key, String value) {
                publishedStrings.put(key, value);
            }
        }) {
            Config.CONFIG.initSendable(testBuilder);
            assertEquals(new HashMap<String, String>(), publishedStrings,
                    "Config.CONFIG must be empty to be on the master branch.");
        }
    }
}
