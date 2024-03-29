#include "yalk/utils/mock.h"

#include "yalk/base/common.h"
#include "yalk/utils/base64.h"
#include "yalk/utils/parser.h"
#include "yalk/utils/random.h"

namespace yalk {

std::pair<std::string, std::string> MockRSAKeyPair() {
  return {
      Base64Decode(
          "MIIBCgKCAQEAx0kmpuYo3bige8UUyixk9QNYi0o4ZpepmAv6InU4oujShA1kNJF8Sczz"
          "Qqmt6k8INte5T0jkOmnkfswR8Tr3xFvdVan+"
          "b4jj3rWVxCpnMHSVCtAv7Pi8NtZ2z97JpUicgZ9Zml0FTzAIow9GfWAsruR/Py/"
          "EHfOreKvtRBYNaSNDsMiZwDB7twaZu1TD5Cvf9v535HuxycUi02Zvm3k8hucMVc3T+"
          "tpebgRN6OwL9WaF4TdndXCZVBlfAOtuW8zAKxGaVswgVvvs2EMhtpaz5O1vjrB6OK/"
          "+AmpltjqOEMJb6Xohib0tNQBuTVLLZfsFUdGuxFW/fxEMlh5YTPh58wIDAQAB"),
      Base64Decode(
          "MIIEpAIBAAKCAQEAx0kmpuYo3bige8UUyixk9QNYi0o4ZpepmAv6InU4oujShA1kNJF8"
          "SczzQqmt6k8INte5T0jkOmnkfswR8Tr3xFvdVan+"
          "b4jj3rWVxCpnMHSVCtAv7Pi8NtZ2z97JpUicgZ9Zml0FTzAIow9GfWAsruR/Py/"
          "EHfOreKvtRBYNaSNDsMiZwDB7twaZu1TD5Cvf9v535HuxycUi02Zvm3k8hucMVc3T+"
          "tpebgRN6OwL9WaF4TdndXCZVBlfAOtuW8zAKxGaVswgVvvs2EMhtpaz5O1vjrB6OK/"
          "+AmpltjqOEMJb6Xohib0tNQBuTVLLZfsFUdGuxFW/"
          "fxEMlh5YTPh58wIDAQABAoIBAF93YIfFbEBC6MZB9CKhv79L2gfoBzrGXp0S58HIC9im"
          "1eoijCVOqI6mGsaNiF3dfF7hWslrbsG45pgDZHj+"
          "OaEcLNyTsep2VkrjDBrMAWDKCHgEoWDRC0Dt3j7YSBrgvPCviHoVI6z6oriSYb8/"
          "6kfh+D8pv7Zle3g0TFY4hjYdsMO4Ly/"
          "lwDAXKhukYAQ4CAg+"
          "aCrUsrljLKTcGhGV3SEepn85CQ2YvAdoE0Ztmkbg0LPT9d77JozeFsgKo5n5zuytSy8r"
          "8lI1cAeHmdSI7R4Rwb9Z1idvvCY9ekpSndAWGrGZFuNvbzOcerAXV9VTRKK56wTPoGO3"
          "MobzHFSJjmkCgYEA8H7pKrtWMghHJWXxlWGjaPsAFYfchkQbI8OuelMPpNSFyB3u46FA"
          "OSFWeI6ScBL4K2/"
          "2gDpCE5SNmAFNldh2AoKklga2oS08EWEbg6h4T9SZ3fokG1pwJKfQgZxp5zPIS54nSZ5"
          "Dr27hyJiM9bov0oTIYp2m0egCW1u2ffZnjCcCgYEA1CIejkodWunlMszTYKtikgFIO69"
          "GAgIx/cGnFnb3E+rfsRf71cEORa/M2r0dQRNne5V19OoSc5SiMiz/"
          "zf05VGAjLIJSGkU03AUrsx/"
          "ZDPoycqzToC5Ij6HfH6ssBKN1QBrCWQEOVyOcHOxsuf+gOe+"
          "UvFrIfWC4riDck9T7J1UCgYEAgvYZMtPhFE29sHfbHmWU/"
          "s3wNclOJS9hOBCPKr2gfznbwCXMuNhOiM3GsM+"
          "ynqU1zzGhIiKXaOI0WNoczcSaPrO1vJzbRtnDJBOlUASlv5MeVuqoTtzoxI091da/NM/"
          "V+pec/VHZuea/"
          "vOSVnhpNgG4eUNVYymV1IxrGPcG4kXECgYEApDcJamLrWI6J1AqFc2cY9iex8Codyr5C"
          "TssTeqt7V1rR8Xi8hdteHgoaThZpyA+qYxvvZTmyMPEVojv9hkuJjsyhTfpQbm+"
          "2nMAP9SkpUAd6+BTb+jr6Bt5rKEvM1oZm5MoSOu47WK+DUUygWlgNg+"
          "v4OtsYKx6tuHZN6s7L4cUCgYBbIIadeWWVizW0m6dRThhWGcafKNrOSyir0zUPwOvNFX"
          "glfT/"
          "dUxmsIx9gv69CDVe0gejWxqp9DISHmUxkFegWQm7o2hFicu6oDijIHDawgUCQeIYqzYB"
          "KA/072eUx0eTwEJz4uczVTzeV5Z+D4YLRiNb2ujL4ZIwd1mGMD7ZwSA==")};
}

ProductInfo MockProductInfo() { return {"YALK", "1.0.0"}; }

UserInfo MockUserInfo() { return {"Yusu Pan", "xxdsox@gmail.com"}; }

HardwareInfo MockHardwareInfo() {
  return {1,
          {{"cpu", "Intel Core i9-12900"},
           {"gpu", "NVIDIA A6000"},
           {"memory", "64GB"},
           {"disk", "1TB SSD"}}};
}

LimitationInfo MockLimitationInfo() {
  return {1600000000,
          1,
          0,
          {"value1", "value2", "value3"},
          LicenseType::kEvaluation,
          1600000000,
          1};
}

json MockAllInfo() {
  json all_info;
  MergeLicenseInfo(MockProductInfo(), MockUserInfo(), MockHardwareInfo(),
                   MockLimitationInfo(), json(), all_info);
  return all_info;
}

LicenseData MockLicenseData() {
  return {MockAllInfo(),
          "c1dc1772ddf0a2837fc5437804e14d07fccf090bedd467a1e45e22e9b69f6a9b"
          "545b41ec55d7a7b47c9cdf1f567a8c471db2f16ae3f0de631c172e0dbe1aa282"
          "c5ea763bf30b340ba7d090e11c1d339c6ba2acc9dcbea3b6496d6864449c85aa"
          "18517af66069cbe901fca3ae28134d8d3e4d7e4ee98a11d12a4cd03c46dbfd90"
          "4a8a0574b70dab045f983b76e18b4a0a4f2f976aca08c584fea214e013be9895"
          "7fd7c052d187324e63005e6a3f4ea5e3a8869a9a76273a36ee6ad0812b5ef178"
          "07b731813dc45e74752b4fff46e24c26e97eda01e943f40b38fa2dac06b5a8d1"
          "6b8df262d1698ba592113c6504c1b2907a85df632150537d1c33fd9b6be7ea2e",
          Base64Decode(
              "JjJrC9BUo0MkQ/nF8IyJUWWKF+tKgAhSTfLM62QeR5LyVY0R9S5KAK/4eciG23VP"
              "8kQhcH7b4ZIIzgi0byapInE9JwodiO+AONi8UrQbQfbWEJhLgRVdx99NKE+CvTl+"
              "r/GIxeH58KlagRSR4APWjqOnD1l69nIZmFoKuCO0WcsjsVSGhNoQ1qMZvlhFkC89"
              "fKyXSxlv2hj4estAA1+txGmiXdUqrkKXmbo4EdglwhXELClBGtceFnGwReyGLzCa"
              "t+lQh0FJT6zDenQ6shLYTolwzDryRF3m1s9oiosxXWzhjKgQor8ATKbp6FLYIM0/"
              "ubSXkORMO82yR2LsA9WV8JnWJ5s1cBpA3HlbxGuuwsxpv9IwLCX4WeBfF0WMCImr"
              "/qp9RmRYTjrVt2NLIZVsJRzRVeo+w4u9BzReepoElRxrda5O3OpKqhTsonxLXSM5"
              "02YtUgh2f9teOEY96NMPGBSkIeKt4p1HiUjQMNV4e7tu12Rqa/CX32HdPjpfqrSc"
              "+34gy3iacj+OSsb9CM2tLcvmV2ktMhBXPV6WYgSWoSUqoDv5PHszmCJUg7gCO3oP"
              "cCRhbB3jlvB64QL4SF+cMiS+NxyefPYmtu9E//PJ4dCk85Pbd4BFf5RfxZFnbOPv"
              "JJvWPf0G6jgivxu/ngmfvp7oiBr5jK6NSWJovcMiBA4=")};
}

ProductInfo RandomProductInfo() {
  auto prob = RandomUniform(0.0, 1.0);
  auto name_length = RandomUniformInt(1, 100);
  if (prob > 0.5) {
    return {RandomString(name_length),
            RandomString(10) + "." + RandomString(10) + "." + RandomString(10)};
  } else if (prob > 0.1) {
    return {RandomString(name_length), RandomString(10)};
  } else {
    return {"", ""};
  }
}

UserInfo RandomUserInfo() {
  auto prob = RandomUniform(0.0, 1.0);
  if (prob > 0.1) {
    auto name_length = RandomUniformInt(1, 100);
    auto email_length = RandomUniformInt(1, 100);
    return {RandomString(name_length), RandomString(email_length)};
  } else {
    return {"", ""};
  }
}

HardwareInfo RandomHardwareInfo() {
  auto prob = RandomUniform(0.0, 1.0);
  if (prob > 0.5) {
    auto cpu_length = RandomUniformInt(1, 100);
    auto gpu_length = RandomUniformInt(1, 100);
    auto memory_length = RandomUniformInt(1, 100);
    auto disk_length = RandomUniformInt(1, 100);
    return {RandomUniformInt(1, 100),
            {{"cpu", RandomString(cpu_length)},
             {"gpu", RandomString(gpu_length)},
             {"memory", RandomString(memory_length)},
             {"disk", RandomString(disk_length)}}};
  } else if (prob > 0.1) {
    return {RandomUniformInt(0, 100),
            {
                {RandomString(10), RandomString(10)},
            }};
  } else {
    return {RandomUniformInt(-100, 100), {}};
  }
}

LimitationInfo RandomLimitationInfo() {
  auto prob = RandomUniform(0.0, 1.0);
  if (prob > 0.1) {
    return {RandomUniformInt(1, 1000000000),
            RandomUniformInt(1, 1000000000),
            RandomUniformInt(1, 1000000000),
            {RandomString(10), RandomString(10), RandomString(10)},
            static_cast<LicenseType>(RandomUniformInt(0, 2)),
            RandomUniformInt(1, 1000000000),
            RandomUniformInt(1, 1000000000)};
  } else {
    return {RandomUniformInt(-100, 100),
            RandomUniformInt(-100, 100),
            RandomUniformInt(-100, 100),
            {RandomString(10), RandomString(10), RandomString(10)},
            static_cast<LicenseType>(RandomUniformInt(-100, 100)),
            RandomUniformInt(-100, 100),
            RandomUniformInt(-100, 100)};
  }
}

json RandomAllInfo() {
  json all_info;
  MergeLicenseInfo(RandomProductInfo(), RandomUserInfo(), RandomHardwareInfo(),
                   RandomLimitationInfo(),
                   json({
                       {RandomString(10), RandomString(10)},
                   }),
                   all_info);
  return all_info;
}

}  // namespace yalk
