LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.NUMERIC_STD.ALL;

ENTITY FSM IS
    GENERIC (N : INTEGER := 2);
    PORT (
        CLK, RESET_BTN, APPLY_BTN : IN STD_LOGIC;
        switches : IN STD_LOGIC_VECTOR(2 * N - 1 DOWNTO 0); -- 3210 (4-bits)
        GainLow, GainLowMid, GainMid, GainHighMid, GainHigh : OUT INTEGER;
        SHOW_GainLowVar, SHOW_GainLowMidVar, SHOW_GainMidVar, SHOW_GainHighMidVar, SHOW_GainHighVar : OUT STD_LOGIC;
        DONE : OUT STD_LOGIC
    );
END FSM;

ARCHITECTURE Behavioral OF FSM IS
    TYPE state_type IS (SHOW_GainLow, SET_GainLow, SHOW_GainLowMid, SET_GainLowMid, SHOW_GainMid, SET_GainMid, SHOW_GainHighMid, SET_GainHighMid, SHOW_GainHigh, SET_GainHigh, FINISHED);
    SIGNAL current_state : state_type := SHOW_GainLow;

BEGIN
    PROCESS (CLK, RESET_BTN, APPLY_BTN)
    BEGIN
		IF RESET_BTN = '1' THEN
				DONE <= '0';
				current_state <= SHOW_GainLow;
        ELSIF falling_edge(APPLY_BTN) THEN
            CASE current_state IS
                WHEN SHOW_GainLow => current_state <= SET_GainLow;
                WHEN SET_GainLow => current_state <= SHOW_GainLowMid;
                WHEN SHOW_GainLowMid => current_state <= SET_GainLowMid;
                WHEN SET_GainLowMid => current_state <= SHOW_GainHigh;
                WHEN SHOW_GainHigh => current_state <= SET_GainHigh;
                WHEN SET_GainHigh => current_state <= SHOW_GainHighMid;
                WHEN SHOW_GainHighMid => current_state <= SET_GainHighMid;
                WHEN SET_GainHighMid => current_state <= SHOW_GainHigh;
                WHEN SHOW_GainHigh => current_state <= SET_GainHigh;
                WHEN SET_GainHigh => current_state <= FINISHED;
                WHEN FINISHED => current_state <= SHOW_GainLow;
            END CASE;
        END IF;

            CASE current_state IS
                WHEN SHOW_GainLow =>
                    SHOW_GainLowVar <= '1';
                WHEN SET_GainLow =>
                    GainLow <= to_integer(signed(switches(2 * N - 1 DOWNTO 0)));
                WHEN SHOW_GainLowMid =>
                    SHOW_GainLowMidVar <= '1';
                WHEN SET_GainLowMid =>
                    GainLowMid <= to_integer(signed(switches(2 * N - 1 DOWNTO 0)));
                WHEN SHOW_GainMid =>
                    SHOW_GainMidVar <= '1';
                WHEN SET_GainMid =>
                    GainMid <= to_integer(signed(switches(2 * N - 1 DOWNTO 0)));
                WHEN SHOW_GainHighMid =>
                    SHOW_GainHighMidVar <= '1';
                WHEN SET_GainHighMid =>
                    GainHighMid <= to_integer(signed(switches(2 * N - 1 DOWNTO 0)));
                WHEN SHOW_GainHigh =>
                    SHOW_GainHighVar <= '1';
                WHEN SET_GainHigh =>
                    GainHigh <= to_integer(signed(switches(2 * N - 1 DOWNTO 0)));
                WHEN FINISHED =>
                    DONE <= '1';
            END CASE;
    END PROCESS;
END Behavioral;