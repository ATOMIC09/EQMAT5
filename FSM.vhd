LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.NUMERIC_STD.ALL;

ENTITY FSM IS
    GENERIC (N : INTEGER := 2);
    PORT (
        CLK, RESET_BTN, APPLY_BTN : IN STD_LOGIC;
        switches : IN STD_LOGIC_VECTOR(2 * N - 1 DOWNTO 0); -- 3210 (4-bits)
        GainLow, GainLowMid, GainMid, GainHighMid, GainHigh : OUT INTEGER;
        DONE : OUT STD_LOGIC
    );
END FSM;

ARCHITECTURE Behavioral OF FSM IS
    TYPE state_type IS (GET_GainLow, GET_GainLowMid, FINISHED);
    SIGNAL current_state : state_type := GET_GainLow;

BEGIN
    PROCESS (CLK, RESET_BTN, APPLY_BTN)
    BEGIN
		IF RESET_BTN = '1' THEN
				DONE <= '0';
				current_state <= GET_GainLow;
        ELSIF falling_edge(APPLY_BTN) THEN
            CASE current_state IS
                WHEN GET_GainLow => current_state <= GET_GainLowMid;
                WHEN GET_GainLowMid => current_state <= FINISHED;
                WHEN FINISHED => current_state <= FINISHED;
            END CASE;
        END IF;

            CASE current_state IS
                WHEN GET_GainLow =>
                    GainLow <= to_integer(signed(switches(2 * N - 1 DOWNTO 0)));
                WHEN GET_GainLowMid =>
                    GainLowMid <= to_integer(signed(switches(2 * N - 1 DOWNTO 0)));
                WHEN FINISHED =>
                    DONE <= '1';
            END CASE;
    END PROCESS;
END Behavioral;