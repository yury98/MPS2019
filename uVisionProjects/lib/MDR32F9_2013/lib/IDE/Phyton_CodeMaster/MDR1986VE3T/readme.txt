���������� �� ���������� ������ ����������� � ����� Phyton CodeMaster.
1. ��������� � ���� CPU.DEF �������� ����������������:
	[Milandr 1986BE3T]
	  FamilyId       = 1986BE3
	  AOPmask        = 0xFFFFFFFF
	  SDriver        = m3
	  adasm          = cortex.dsm
	  prcOption      = arm7tdmi
	  mcaOption      = M1
	  stup_suffix    = M0
	  MEMfile        = Milandr\\1986BE3
	  REMAPfile      = 1986BE3
	  LDfile         = arm7tdmi
	  SFRfile        = Milandr\\1986BE3\\1986be3t
	  CmcStartupFile = Milandr\\1986BE1\\startup_MDR1986VE3
	  PDW            = Generic ARM7TDMI
	  FPU_enabled    = 0

2. ����������� ����� 1986BE3.mem, 1986BE3_flash_128K.inc, 1986BE3_ram_32K_16K.inc 
   � ���������� MEM/Milandr.

3. ����������� ����� 1986be3t.sfr, CortexM0.SFR � ���������� SFR/Milandr.

4. �������� � ���� POD/JEM-ARM-V2/Milandr/FAMILY_CM1.CGF:
	[[Milandr 1986BE3]]
	 ScriptFolder=Flash/Milandr/1986BE3x
	 FamilyId=1986BE3

5. �������� � ���� POD/JEM-ARM-V2/Milandr/CHIPS_CM1.INC:
	DEFINE_CHIP(0x00000000,Milandr 1986BE3T)
	  ConfFields=Milandr\\1986BE3\\1986BE3x.fld

6. ����������� � ����� POD/JEM-ARM-V2/Milandr/ ������� FLD/1986BE3.

7. ����������� � ����� Flash/Milandr ������� Flash/1986BE3.




