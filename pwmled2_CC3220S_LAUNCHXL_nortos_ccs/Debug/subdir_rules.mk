################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"/Applications/ti/ccs1250/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=vfplib -me --include_path="/Users/rajivdialani/workspace_v12/pwmled2_CC3220S_LAUNCHXL_nortos_ccs" --include_path="/Users/rajivdialani/workspace_v12/pwmled2_CC3220S_LAUNCHXL_nortos_ccs/Debug" --include_path="/Users/rajivdialani/ti/simplelink_cc32xx_sdk_4_30_00_06/source" --include_path="/Users/rajivdialani/ti/simplelink_cc32xx_sdk_4_30_00_06/kernel/nortos" --include_path="/Users/rajivdialani/ti/simplelink_cc32xx_sdk_4_30_00_06/kernel/nortos/posix" --include_path="/Applications/ti/ccs1250/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS/include" --define=NORTOS_SUPPORT -g --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" --include_path="/Users/rajivdialani/workspace_v12/pwmled2_CC3220S_LAUNCHXL_nortos_ccs/Debug/syscfg" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

build-642115183: ../pwmled2.syscfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: SysConfig'
	"/Users/rajivdialani/ti/sysconfig_1_12_0/sysconfig_cli.sh" -s "/Users/rajivdialani/ti/simplelink_cc32xx_sdk_4_30_00_06/.metadata/product.json" --script "/Users/rajivdialani/workspace_v12/pwmled2_CC3220S_LAUNCHXL_nortos_ccs/pwmled2.syscfg" -o "syscfg" --compiler ccs
	@echo 'Finished building: "$<"'
	@echo ' '

syscfg/ti_drivers_config.c: build-642115183 ../pwmled2.syscfg
syscfg/ti_drivers_config.h: build-642115183
syscfg/ti_utils_build_linker.cmd.genlibs: build-642115183
syscfg/syscfg_c.rov.xs: build-642115183
syscfg/ti_utils_runtime_model.gv: build-642115183
syscfg/ti_utils_runtime_Makefile: build-642115183
syscfg/: build-642115183

syscfg/%.obj: ./syscfg/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"/Applications/ti/ccs1250/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=vfplib -me --include_path="/Users/rajivdialani/workspace_v12/pwmled2_CC3220S_LAUNCHXL_nortos_ccs" --include_path="/Users/rajivdialani/workspace_v12/pwmled2_CC3220S_LAUNCHXL_nortos_ccs/Debug" --include_path="/Users/rajivdialani/ti/simplelink_cc32xx_sdk_4_30_00_06/source" --include_path="/Users/rajivdialani/ti/simplelink_cc32xx_sdk_4_30_00_06/kernel/nortos" --include_path="/Users/rajivdialani/ti/simplelink_cc32xx_sdk_4_30_00_06/kernel/nortos/posix" --include_path="/Applications/ti/ccs1250/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS/include" --define=NORTOS_SUPPORT -g --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --preproc_with_compile --preproc_dependency="syscfg/$(basename $(<F)).d_raw" --include_path="/Users/rajivdialani/workspace_v12/pwmled2_CC3220S_LAUNCHXL_nortos_ccs/Debug/syscfg" --obj_directory="syscfg" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


