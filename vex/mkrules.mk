# VEXcode mkrules.mk 2019_03_26_01

# compile C files
$(BUILD)/%.o: %.c $(SRC_H)
	$(Q)$(MKDIR)
	$(ECHO) "CC  $<"
	$(Q)$(CC) $(CFLAGS) $(INC) -c -o $@ $<
	
# compile C++ files
$(BUILD)/%.o: %.cpp $(SRC_H) $(SRC_A)
	$(Q)$(MKDIR)
ifeq ($(CLANG_TIDY), 1)
	$(ECHO) "CXX tidy $<"
	$(Q)$(CXX_TIDY) $< -- $(CXX_FLAGS) $(INC)  2>/dev/null	
endif
	$(ECHO) "CXX $<"
	$(Q)$(CXX) $(CXX_FLAGS) $(INC) -c -o $@ $<
	
	
# create executable 
$(BUILD)/$(PROJECT).elf: $(OBJ)
	$(ECHO) "LINK $@"
	$(Q)$(LINK) $(LNK_FLAGS) -o $@ $^ $(LIBS)
	$(Q)$(SIZE) $@


# create binary 
$(BUILD)/$(PROJECT).bin: $(BUILD)/$(PROJECT).elf
	$(Q)$(OBJCOPY) -O binary $(BUILD)/$(PROJECT).elf $(BUILD)/$(PROJECT).bin

# create archive
$(BUILD)/$(PROJECTLIB).a: $(OBJ)
	$(Q)$(ARCH) $(ARCH_FLAGS) $@ $^

# clean project
clean:
	$(info clean project)
	$(Q)$(CLEAN)
