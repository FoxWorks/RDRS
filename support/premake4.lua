-- Create standalone solution
if RDRS_STANDALONE ~= false then
  SIMC_STANDALONE = false
  solution "rdrs"
     dofile("./../external/simc/support/premake4_common.lua")
     dofile("./../external/simc/support/premake4.lua")
end


--------------------------------------------------------------------------------
-- RDRS documentation
--------------------------------------------------------------------------------
newaction {
   trigger     = "rdrsdoc",
   description = "Generate documentation for RDRS",
   execute     = function()
     if EVDS_STANDALONE ~= false
     then os.chdir("../doc")
     else os.chdir("./../external/evds/doc")
     end
     os.execute("doxygen")
   end
}
if _ACTION == "rdrsdoc" then return end


--------------------------------------------------------------------------------
-- Realtime Digital Radio Simulator
--------------------------------------------------------------------------------
project "rdrs"
   library()
   language "C"
   includedirs { "../include",
                 "../external/simc/include" }
   files { "../source/rdrs_core/**",
           "../include/**" }
   defines { "RDRS_LIBRARY", "SIMC_LIBRARY" }
   links { "simc" }


--------------------------------------------------------------------------------
-- Tutorials
--------------------------------------------------------------------------------
project "rdrs_test"
   kind "ConsoleApp"
   language "C"
   includedirs { "../include",
                 "../external/simc/include" }
   files { "../source/rdrs_tests/rdrs_test.c"  }
   links { "rdrs" }
   
   configuration { "not windows" }
      links { "simc", "tinyxml" }
