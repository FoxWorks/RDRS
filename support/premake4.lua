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
   uuid "1418E57D-CB20-5542-8D85-B90995D5B8BE"
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
if RDRS_STANDALONE ~= false then
   project "rdrs_test"
      uuid "9DFD9881-6E95-A14C-A73C-6AF89CDA9338"
      kind "ConsoleApp"
      language "C"
      includedirs { "../include",
                    "../external/simc/include" }
      files { "../source/rdrs_tests/rdrs_test.c"  }
      links { "rdrs" }
      
      configuration { "not windows" }
         links { "simc", "tinyxml" }
end
