/* Flydeck.app/Contents/MacOS/Flydeck: start the bundled Python inside this process.
 *
 * Built by `uv run flydeck build-app` (flydeck/build_app.py). The app's own executable runs
 * Python in-process (libpython from Contents/Resources/python), so macOS sees one app called
 * Flydeck: its Dock icon, its menu bar name, and the app that asks for Bluetooth permission.
 * It runs `python -m flydeck`; everything Flydeck starts (the hub, demos, robot checks) uses
 * Contents/Resources/python/bin/python3 and descends from this process.
 *
 * Nothing is written inside the bundle: compiled .pyc files go to
 * ~/Library/Caches/com.neuraffica.flydeck/pycache.
 */
#define PY_SSIZE_T_CLEAN
#include <Python.h>

#include <limits.h>
#include <mach-o/dyld.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void fail(const char *what) {
    fprintf(stderr, "Flydeck: %s\n", what);
    exit(70);
}

static void dirname_inplace(char *path) {
    char *slash = strrchr(path, '/');
    if (slash && slash != path) *slash = '\0';
}

static void check(PyStatus status, PyConfig *config) {
    if (PyStatus_Exception(status)) {
        PyConfig_Clear(config);
        Py_ExitStatusException(status);
    }
}

int main(int argc, char **argv) {
    char raw[PATH_MAX], exe[PATH_MAX];
    uint32_t size = sizeof(raw);
    if (_NSGetExecutablePath(raw, &size) != 0 || realpath(raw, exe) == NULL)
        fail("cannot find my own path");

    char contents[PATH_MAX], app[PATH_MAX], home[PATH_MAX], python[PATH_MAX];
    char cache[PATH_MAX];
    snprintf(contents, sizeof(contents), "%s", exe);
    dirname_inplace(contents); /* .../Contents/MacOS */
    dirname_inplace(contents); /* .../Contents */
    snprintf(app, sizeof(app), "%s", contents);
    dirname_inplace(app); /* .../Flydeck.app */
    snprintf(home, sizeof(home), "%s/Resources/python", contents);
    snprintf(python, sizeof(python), "%s/bin/python3", home);

    const char *user_home = getenv("HOME");
    if (user_home && *user_home)
        snprintf(cache, sizeof(cache), "%s/Library/Caches/com.neuraffica.flydeck/pycache",
                 user_home);
    else
        snprintf(cache, sizeof(cache), "/tmp/com.neuraffica.flydeck-pycache");

    /* What the children inherit: the bundled Python, no user or shell Python settings. */
    unsetenv("PYTHONHOME");
    unsetenv("PYTHONPATH");
    unsetenv("PYTHONSTARTUP");
    unsetenv("PYTHONSAFEPATH");
    unsetenv("VIRTUAL_ENV");
    setenv("PYTHONNOUSERSITE", "1", 1);
    setenv("PYTHONPYCACHEPREFIX", cache, 1);
    setenv("FLYDECK_BUNDLE", app, 1);

    PyConfig config;
    PyConfig_InitIsolatedConfig(&config);
    check(PyConfig_SetBytesString(&config, &config.home, home), &config);
    check(PyConfig_SetBytesString(&config, &config.executable, python), &config);
    check(PyConfig_SetBytesString(&config, &config.pycache_prefix, cache), &config);
    check(PyConfig_SetString(&config, &config.run_module, L"flydeck"), &config);
    config.install_signal_handlers = 1;

    /* Finder used to add -psn_... to argv; drop it. Everything else goes to flydeck. */
    char **args = calloc((size_t)argc + 1, sizeof(char *));
    if (args == NULL) fail("out of memory");
    int n = 0;
    for (int i = 0; i < argc; i++) {
        if (i > 0 && strncmp(argv[i], "-psn_", 5) == 0) continue;
        args[n++] = argv[i];
    }
    check(PyConfig_SetBytesArgv(&config, n, args), &config);
    free(args);

    check(Py_InitializeFromConfig(&config), &config);
    PyConfig_Clear(&config);
    return Py_RunMain();
}
