/**
 * Copyright (c) 2026, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * This software is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#include <stdint.h>
#include <stdio.h>

#include <mujoco/mujoco.h>

static void listLinkedPlugins()
{
  // check and print plugins that are linked directly into the executable
  int nplugin = mjp_pluginCount();
  if (nplugin)
  {
    std::printf("Built-in plugins:\n");
    for (int i = 0; i < nplugin; ++i)
    {
      std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
    }
  }
  else
    std::printf("!!!!!!!!!!!!!!!!!!!!!!! No plugins found\n");
}

static void loadPlugins(std::string plugin_dir)
{
  mj_loadAllPluginLibraries(
      plugin_dir.c_str(), +[](const char* filename, int first, int count) {
        std::printf("Plugins registered by library '%s':\n", filename);
        for (int i = first; i < first + count; ++i)
        {
          std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
        }
      });
}

static void loadCustomPlugins()
{
  char* plptr = std::getenv("mujoco_DIR");
  std::string plugin_dir = "";
  if (plptr)
  {
    plugin_dir = std::string(plptr) + "/bin/mujoco_plugin";
  }
  else
    plugin_dir = "/usr/include/bin/mujoco_plugin";
  printf("Looking for plugins at %s\n", plugin_dir.c_str());
  loadPlugins(plugin_dir);
}

int32_t main(int32_t, char**)
{
  // recommended version check
  if (mjVERSION_HEADER != mj_version())
    printf("Versions don't match wtf\n");
  else
    printf("Versions match whoo hooo\n");

  loadCustomPlugins();
  listLinkedPlugins();

  return (0);
}
