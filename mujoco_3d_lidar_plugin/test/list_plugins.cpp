#include <stdint.h>
#include <stdio.h>

#include <mujoco/mujoco.h>

static void listLinkedPlugins() {
  // check and print plugins that are linked directly into the executable
  int nplugin = mjp_pluginCount();
  if (nplugin) {
    std::printf("Built-in plugins:\n");
    for (int i = 0; i < nplugin; ++i) {
      std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
    }
  } else
    std::printf("!!!!!!!!!!!!!!!!!!!!!!! No plugins found\n");
}

static void loadPlugins(std::string plugin_dir) {
  mj_loadAllPluginLibraries(
      plugin_dir.c_str(), +[](const char *filename, int first, int count) {
        std::printf("Plugins registered by library '%s':\n", filename);
        for (int i = first; i < first + count; ++i) {
          std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
        }
      });
}

static void loadCustomPlugins() {
  char *plptr = std::getenv("mujoco_DIR");
  std::string plugin_dir = "";
  if (plptr) {
    plugin_dir = std::string(plptr) + "/bin/mujoco_plugin";
  } else
    plugin_dir = "/usr/include/bin/mujoco_plugin";
  printf("Looking for plugins at %s\n", plugin_dir.c_str());
  loadPlugins(plugin_dir);
}

int32_t main(int32_t, char **) {
  // recommended version check
  if (mjVERSION_HEADER != mj_version())
    printf("Versions don't match wtf\n");
  else
    printf("Versions match whoo hooo\n");

  loadCustomPlugins();
  listLinkedPlugins();

  return (0);
}
