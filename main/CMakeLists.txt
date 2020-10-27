idf_component_register(SRCS "hello_world_main.c"
                    INCLUDE_DIRS "."
                    REQUIRES soc nvs_flash ulp)

set(ulp_app_name ulp_${COMPONENT_NAME})

set(ulp_s_sources "ulp/ulp.S")

set(ulp_exp_dep_srcs "hello_world_main.c")

ulp_embed_binary(${ulp_app_name} "${ulp_s_sources}" "${ulp_exp_dep_srcs}")