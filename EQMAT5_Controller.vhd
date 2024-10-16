library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

entity EQMAT5_Controller is
    port (
        CLK : in std_logic;
        RESET_BTN, APPLY_BTN : in std_logic;
        switches : in std_logic_vector(7 downto 0); -- Example: 8 switches for 5 gains (FSM controls this)
        DONE : out std_logic; -- Finished processing all bands
        a0, a1, a2, b1, b2 : out real -- Biquad coefficients output
    );
end EQMAT5_Controller;

architecture Behavioral of EQMAT5_Controller is
    -- Define constant center frequencies for each band
    constant Fc_Low : real := 50.0;
    constant Fc_LowMid : real := 200.0;
    constant Fc_Mid : real := 1500.0;
    constant Fc_HighMid : real := 3000.0;
    constant Fc_High : real := 10000.0;
    constant Fs : real := 96000.0;
    constant Q : real := 1.0;
    
    -- Signals to receive gain values from FSM
    signal GainLow, GainLowMid, GainMid, GainHighMid, GainHigh : integer;

    -- Internal signals for biquad inputs and outputs
    signal Fc : real;
    signal a0_temp, a1_temp, a2_temp, b1_temp, b2_temp : real;

    -- Instantiate FSM (connect to FSM.vhd)
    component FSM
        port (
            CLK : in std_logic;
            RESET_BTN : in std_logic;
            APPLY_BTN : in std_logic;
            switches : in std_logic_vector(7 downto 0);
            GainLow, GainLowMid, GainMid, GainHighMid, GainHigh : out integer;
            SHOW_GainLowVar, SHOW_GainLowMidVar, SHOW_GainMidVar, SHOW_GainHighMidVar, SHOW_GainHighVar : out std_logic;
            DONE : out std_logic
        );
    end component;
    
    -- Instantiate Biquad (connect to Biquad.vhd)
    component Biquad
        port (
            GainLow, GainLowMid, GainMid, GainHighMid, GainHigh : in integer;
            Fc, Fs, Q : in real;
            CLK : in std_logic;
            a0, a1, a2, b1, b2 : out real
        );
    end component;

begin

    -- FSM instantiation: pass the gain values to FSM
    FSM_inst : FSM
        port map (
            CLK => CLK,
            RESET_BTN => RESET_BTN,
            APPLY_BTN => APPLY_BTN,
            switches => switches,
            GainLow => GainLow,
            GainLowMid => GainLowMid,
            GainMid => GainMid,
            GainHighMid => GainHighMid,
            GainHigh => GainHigh,
            SHOW_GainLowVar => open,
            SHOW_GainLowMidVar => open,
            SHOW_GainMidVar => open,
            SHOW_GainHighMidVar => open,
            SHOW_GainHighVar => open,
            DONE => DONE
        );

    -- Process to select the correct Fc based on the state from FSM
    process(CLK)
    begin
        if rising_edge(CLK) then
            -- Depending on which gain is being adjusted, assign corresponding Fc
            if GainLow /= 0 then
                Fc <= Fc_Low;
            elsif GainLowMid /= 0 then
                Fc <= Fc_LowMid;
            elsif GainMid /= 0 then
                Fc <= Fc_Mid;
            elsif GainHighMid /= 0 then
                Fc <= Fc_HighMid;
            elsif GainHigh /= 0 then
                Fc <= Fc_High;
            end if;
        end if;
    end process;

    -- Biquad instantiation: calculate coefficients based on current gain and Fc
    Biquad_inst : Biquad
        port map (
            GainLow => GainLow,
            GainLowMid => GainLowMid,
            GainMid => GainMid,
            GainHighMid => GainHighMid,
            GainHigh => GainHigh,
            Fc => Fc,
            Fs => Fs,
            Q => Q,
            CLK => CLK,
            a0 => a0_temp,
            a1 => a1_temp,
            a2 => a2_temp,
            b1 => b1_temp,
            b2 => b2_temp
        );

    -- Assign the calculated coefficients to output ports
    a0 <= a0_temp;
    a1 <= a1_temp;
    a2 <= a2_temp;
    b1 <= b1_temp;
    b2 <= b2_temp;

end Behavioral;