library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity DISP_4X7SEG_DEMO is
  generic( 
    CLK_HZ  : natural := 24000000
  );
  port(
    CLK      : in std_logic;
    nRST     : in std_logic;
    SEGMENTS : out std_logic_vector(7 downto 0);
    DIGITS   : out std_logic_vector(3 downto 0) 
 );
end DISP_4X7SEG_DEMO;

architecture SYNTH of DISP_4X7SEG_DEMO is
  constant NUM_DIGITS : natural := 4;
  constant CNT_MAX1   : natural := (CLK_HZ/400)-1;
  constant CNT_MAX2   : natural := (CLK_HZ/10)-1;

  type bcd_counter_t is array(0 to NUM_DIGITS-1) of integer range 0 to 9;
  signal bcd_count  : bcd_counter_t; 

  signal data_buf     : std_logic_vector(7 downto 0);
  signal digits_sel   : std_logic_vector(NUM_DIGITS-1 downto 0);
  signal digit_index  : natural range 0 to NUM_DIGITS-1 := 0;

  subtype nibble is unsigned(3 downto 0);
  -- This function implements a BCD to 7-Segment decoder.
  function BCD2SEG7( data: nibble ) return std_logic_vector is
     variable seg7bits : std_logic_vector(6 downto 0);
  begin
     case data is
        when "0000" => seg7bits := "1000000"; -- 0
        when "0001" => seg7bits := "1111001"; -- 1
        when "0010" => seg7bits := "0100100"; -- 2
        when "0011" => seg7bits := "0110000"; -- 3
        when "0100" => seg7bits := "0011001"; -- 4
        when "0101" => seg7bits := "0010010"; -- 5
        when "0110" => seg7bits := "0000010"; -- 6
        when "0111" => seg7bits := "1111000"; -- 7
        when "1000" => seg7bits := "0000000"; -- 8
        when "1001" => seg7bits := "0010000"; -- 9
        when others => seg7bits := "1111111"; -- off
     end case;
     return seg7bits;
  end BCD2SEG7;

begin

  -- This process implements a N-digit BCD counter.
  process (nRST,CLK)
    variable wait_cnt    : natural range 0 to CNT_MAX2 := 0;
    variable clk_enabled : boolean;
    variable carry       : boolean; 
  begin
    if nRST = '0' then
      wait_cnt := 0;
      for i in bcd_count'range loop
         bcd_count(i) <= 0;
      end loop;
    elsif rising_edge(CLK) then 
       if wait_cnt = CNT_MAX2 then
          wait_cnt := 0;
          clk_enabled := true;
       else 
          wait_cnt := wait_cnt + 1;
          clk_enabled := false;
       end if;
       if clk_enabled then
          carry := true;
          for i in 0 to NUM_DIGITS-1 loop
             if carry then
                if bcd_count(i)=9 then
                   bcd_count(i) <= 0;
                   carry := true;
                else
                   bcd_count(i) <= bcd_count(i)+1;
                   carry := false;
                end if;
             else
                carry := false;
             end if;
          end loop;
       end if;
    end if;
  end process; 

  -- This process implements a N-digit 7-segment driver
  -- using time-multiplexing.
  process (nRST, CLK) 
    variable wait_cnt    : natural range 0 to CNT_MAX1 := 0;
    variable clk_enabled : boolean;
    variable bcd_value   : unsigned(3 downto 0);
  begin
    if nRST = '0' then
       wait_cnt := 0;
       data_buf    <= x"00";
       digits_sel  <= (others => '0');
       digit_index <= 0;
    elsif rising_edge(CLK) then     
       if wait_cnt = CNT_MAX1 then
          wait_cnt := 0;
          clk_enabled := true;
       else 
          wait_cnt := wait_cnt + 1;
          clk_enabled := false;
       end if;
       if clk_enabled then
          if digit_index = NUM_DIGITS-1 then 
             digit_index <= 0;
          else 
             digit_index <= digit_index + 1;
          end if;
          for i in 0 to NUM_DIGITS-1 loop
             if i = digit_index then
               digits_sel(i) <= '1';
             else 
               digits_sel(i) <= '0';
             end if;
          end loop;
          bcd_value := to_unsigned(bcd_count(digit_index),4);
          data_buf <= '0' & BCD2SEG7( bcd_value );
       end if;
    end if;
  end process;

  DIGITS   <= (others => '1') when nRST = '0' else (not digits_sel);
  SEGMENTS <= not data_buf; -- for common-anode 7-segment LEDs

end SYNTH;