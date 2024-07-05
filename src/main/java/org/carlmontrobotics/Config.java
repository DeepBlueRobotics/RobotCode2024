package org.carlmontrobotics;

import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public abstract class Config implements Sendable {
    public static final Config CONFIG = new Config() {
        {
            // Override config settings here, like this:
            // this.exampleFlagEnabled = true;

            // NOTE: PRs with overrides will NOT be merged because we don't want them
            // polluting the master branch.
            // Feel free to add them when testing, but remove them before pushing.
        }
    };

    // Add additional config settings by declaring a protected field, and...
    protected boolean exampleFlagEnabled = false;
    protected boolean swimShady = false;
    protected boolean setupSysId = false;
    protected boolean useSmartDashboardControl = false; // whether to control arm position + rpm of
                                                       // outtake through SmartDashboard
                                                       // Note: disables joystick control of arm and
                                                       // outtake command if
                                                       // using SmartDashboard

    // ...a public getter starting with "is" for booleans or "get" for other types.
    // Do NOT remove this example. It is used by unit tests.

    public boolean isExampleFlagEnabled() {
        return exampleFlagEnabled;
    }

    public boolean isSwimShady() {
        return swimShady;
    }

    public boolean isSysIdTesting() {
        return setupSysId;
    }

    public boolean useSmartDashboardControl() {
        return useSmartDashboardControl;
    }

    // --- For clarity, place additional config settings ^above^ this line ---

    private static class MethodResult {
        String methodName = null;
        Object retVal = null;
        Object defaultRetVal = null;

        MethodResult(String name, Object retVal, Object defaultRetval) {
            this.methodName = name;
            this.retVal = retVal;
            this.defaultRetVal = defaultRetval;
        }
    }

    private List<MethodResult> getMethodResults() {
        var methodResults = new ArrayList<MethodResult>();
        var defaultConfig = new Config() {
        };
        for (Method m : Config.class.getDeclaredMethods()) {
            var name = m.getName();
            if (!Modifier.isPublic(m.getModifiers()) || m.isSynthetic() || m.getParameterCount() != 0
                    || !name.matches("^(get|is)[A-Z].*")) {
                continue;
            }
            Object retVal = null;
            try {
                retVal = m.invoke(this);
            } catch (Exception ex) {
                retVal = ex;
            }
            Object defaultRetVal = null;
            try {
                defaultRetVal = m.invoke(defaultConfig);
            } catch (Exception ex) {
                defaultRetVal = ex;
            }
            methodResults.add(new MethodResult(name, retVal, defaultRetVal));
        }
        return methodResults;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        getMethodResults().forEach(mr -> {
            if (!mr.retVal.equals(mr.defaultRetVal)) {
                builder.publishConstString("%s()".formatted(mr.methodName),
                        String.format("%s (default is %s)", mr.retVal, mr.defaultRetVal));
            }
        });
    }

    @Override
    public String toString() {
        StringBuilder stringBuilder = new StringBuilder();
        getMethodResults().forEach(mr -> {
            if (!mr.retVal.equals(mr.defaultRetVal)) {
                stringBuilder.append(
                        String.format("%s() returns %s (default is %s)", mr.methodName, mr.retVal, mr.defaultRetVal));
            }
        });
        if (stringBuilder.isEmpty()) {
            stringBuilder.append("Using default config values");
        } else {
            stringBuilder.insert(0, "WARNING: USING OVERRIDDEN CONFIG VALUES\n");
        }
        return stringBuilder.toString();
    }
}
