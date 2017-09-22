;; Auto-generated. Do not edit!


(when (boundp 'keyboard::Key)
  (if (not (find-package "KEYBOARD"))
    (make-package "KEYBOARD"))
  (shadow 'Key (find-package "KEYBOARD")))
(unless (find-package "KEYBOARD::KEY")
  (make-package "KEYBOARD::KEY"))

(in-package "ROS")
;;//! \htmlinclude Key.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*KEY_UNKNOWN*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_UNKNOWN* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_UNKNOWN* 0)
(intern "*KEY_FIRST*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_FIRST* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_FIRST* 0)
(intern "*KEY_BACKSPACE*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_BACKSPACE* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_BACKSPACE* 8)
(intern "*KEY_TAB*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_TAB* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_TAB* 9)
(intern "*KEY_CLEAR*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_CLEAR* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_CLEAR* 12)
(intern "*KEY_RETURN*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_RETURN* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_RETURN* 13)
(intern "*KEY_PAUSE*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_PAUSE* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_PAUSE* 19)
(intern "*KEY_ESCAPE*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_ESCAPE* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_ESCAPE* 27)
(intern "*KEY_SPACE*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_SPACE* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_SPACE* 32)
(intern "*KEY_EXCLAIM*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_EXCLAIM* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_EXCLAIM* 33)
(intern "*KEY_QUOTEDBL*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_QUOTEDBL* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_QUOTEDBL* 34)
(intern "*KEY_HASH*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_HASH* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_HASH* 35)
(intern "*KEY_DOLLAR*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_DOLLAR* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_DOLLAR* 36)
(intern "*KEY_AMPERSAND*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_AMPERSAND* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_AMPERSAND* 38)
(intern "*KEY_QUOTE*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_QUOTE* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_QUOTE* 39)
(intern "*KEY_LEFTPAREN*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_LEFTPAREN* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_LEFTPAREN* 40)
(intern "*KEY_RIGHTPAREN*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_RIGHTPAREN* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_RIGHTPAREN* 41)
(intern "*KEY_ASTERISK*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_ASTERISK* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_ASTERISK* 42)
(intern "*KEY_PLUS*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_PLUS* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_PLUS* 43)
(intern "*KEY_COMMA*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_COMMA* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_COMMA* 44)
(intern "*KEY_MINUS*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_MINUS* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_MINUS* 45)
(intern "*KEY_PERIOD*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_PERIOD* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_PERIOD* 46)
(intern "*KEY_SLASH*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_SLASH* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_SLASH* 47)
(intern "*KEY_0*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_0* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_0* 48)
(intern "*KEY_1*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_1* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_1* 49)
(intern "*KEY_2*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_2* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_2* 50)
(intern "*KEY_3*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_3* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_3* 51)
(intern "*KEY_4*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_4* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_4* 52)
(intern "*KEY_5*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_5* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_5* 53)
(intern "*KEY_6*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_6* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_6* 54)
(intern "*KEY_7*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_7* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_7* 55)
(intern "*KEY_8*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_8* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_8* 56)
(intern "*KEY_9*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_9* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_9* 57)
(intern "*KEY_COLON*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_COLON* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_COLON* 58)
(intern "*KEY_SEMICOLON*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_SEMICOLON* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_SEMICOLON* 59)
(intern "*KEY_LESS*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_LESS* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_LESS* 60)
(intern "*KEY_EQUALS*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_EQUALS* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_EQUALS* 61)
(intern "*KEY_GREATER*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_GREATER* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_GREATER* 62)
(intern "*KEY_QUESTION*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_QUESTION* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_QUESTION* 63)
(intern "*KEY_AT*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_AT* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_AT* 64)
(intern "*KEY_LEFTBRACKET*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_LEFTBRACKET* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_LEFTBRACKET* 91)
(intern "*KEY_BACKSLASH*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_BACKSLASH* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_BACKSLASH* 92)
(intern "*KEY_RIGHTBRACKET*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_RIGHTBRACKET* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_RIGHTBRACKET* 93)
(intern "*KEY_CARET*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_CARET* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_CARET* 94)
(intern "*KEY_UNDERSCORE*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_UNDERSCORE* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_UNDERSCORE* 95)
(intern "*KEY_BACKQUOTE*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_BACKQUOTE* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_BACKQUOTE* 96)
(intern "*KEY_A*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_A* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_A* 97)
(intern "*KEY_B*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_B* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_B* 98)
(intern "*KEY_C*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_C* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_C* 99)
(intern "*KEY_D*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_D* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_D* 100)
(intern "*KEY_E*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_E* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_E* 101)
(intern "*KEY_F*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_F* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_F* 102)
(intern "*KEY_G*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_G* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_G* 103)
(intern "*KEY_H*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_H* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_H* 104)
(intern "*KEY_I*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_I* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_I* 105)
(intern "*KEY_J*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_J* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_J* 106)
(intern "*KEY_K*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_K* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_K* 107)
(intern "*KEY_L*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_L* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_L* 108)
(intern "*KEY_M*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_M* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_M* 109)
(intern "*KEY_N*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_N* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_N* 110)
(intern "*KEY_O*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_O* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_O* 111)
(intern "*KEY_P*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_P* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_P* 112)
(intern "*KEY_Q*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_Q* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_Q* 113)
(intern "*KEY_R*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_R* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_R* 114)
(intern "*KEY_S*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_S* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_S* 115)
(intern "*KEY_T*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_T* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_T* 116)
(intern "*KEY_U*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_U* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_U* 117)
(intern "*KEY_V*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_V* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_V* 118)
(intern "*KEY_W*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_W* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_W* 119)
(intern "*KEY_X*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_X* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_X* 120)
(intern "*KEY_Y*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_Y* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_Y* 121)
(intern "*KEY_Z*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_Z* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_Z* 122)
(intern "*KEY_DELETE*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_DELETE* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_DELETE* 127)
(intern "*KEY_WORLD_0*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_0* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_0* 160)
(intern "*KEY_WORLD_1*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_1* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_1* 161)
(intern "*KEY_WORLD_2*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_2* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_2* 162)
(intern "*KEY_WORLD_3*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_3* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_3* 163)
(intern "*KEY_WORLD_4*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_4* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_4* 164)
(intern "*KEY_WORLD_5*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_5* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_5* 165)
(intern "*KEY_WORLD_6*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_6* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_6* 166)
(intern "*KEY_WORLD_7*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_7* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_7* 167)
(intern "*KEY_WORLD_8*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_8* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_8* 168)
(intern "*KEY_WORLD_9*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_9* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_9* 169)
(intern "*KEY_WORLD_10*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_10* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_10* 170)
(intern "*KEY_WORLD_11*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_11* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_11* 171)
(intern "*KEY_WORLD_12*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_12* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_12* 172)
(intern "*KEY_WORLD_13*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_13* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_13* 173)
(intern "*KEY_WORLD_14*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_14* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_14* 174)
(intern "*KEY_WORLD_15*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_15* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_15* 175)
(intern "*KEY_WORLD_16*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_16* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_16* 176)
(intern "*KEY_WORLD_17*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_17* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_17* 177)
(intern "*KEY_WORLD_18*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_18* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_18* 178)
(intern "*KEY_WORLD_19*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_19* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_19* 179)
(intern "*KEY_WORLD_20*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_20* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_20* 180)
(intern "*KEY_WORLD_21*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_21* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_21* 181)
(intern "*KEY_WORLD_22*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_22* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_22* 182)
(intern "*KEY_WORLD_23*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_23* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_23* 183)
(intern "*KEY_WORLD_24*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_24* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_24* 184)
(intern "*KEY_WORLD_25*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_25* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_25* 185)
(intern "*KEY_WORLD_26*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_26* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_26* 186)
(intern "*KEY_WORLD_27*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_27* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_27* 187)
(intern "*KEY_WORLD_28*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_28* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_28* 188)
(intern "*KEY_WORLD_29*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_29* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_29* 189)
(intern "*KEY_WORLD_30*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_30* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_30* 190)
(intern "*KEY_WORLD_31*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_31* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_31* 191)
(intern "*KEY_WORLD_32*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_32* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_32* 192)
(intern "*KEY_WORLD_33*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_33* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_33* 193)
(intern "*KEY_WORLD_34*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_34* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_34* 194)
(intern "*KEY_WORLD_35*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_35* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_35* 195)
(intern "*KEY_WORLD_36*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_36* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_36* 196)
(intern "*KEY_WORLD_37*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_37* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_37* 197)
(intern "*KEY_WORLD_38*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_38* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_38* 198)
(intern "*KEY_WORLD_39*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_39* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_39* 199)
(intern "*KEY_WORLD_40*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_40* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_40* 200)
(intern "*KEY_WORLD_41*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_41* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_41* 201)
(intern "*KEY_WORLD_42*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_42* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_42* 202)
(intern "*KEY_WORLD_43*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_43* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_43* 203)
(intern "*KEY_WORLD_44*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_44* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_44* 204)
(intern "*KEY_WORLD_45*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_45* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_45* 205)
(intern "*KEY_WORLD_46*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_46* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_46* 206)
(intern "*KEY_WORLD_47*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_47* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_47* 207)
(intern "*KEY_WORLD_48*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_48* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_48* 208)
(intern "*KEY_WORLD_49*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_49* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_49* 209)
(intern "*KEY_WORLD_50*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_50* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_50* 210)
(intern "*KEY_WORLD_51*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_51* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_51* 211)
(intern "*KEY_WORLD_52*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_52* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_52* 212)
(intern "*KEY_WORLD_53*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_53* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_53* 213)
(intern "*KEY_WORLD_54*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_54* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_54* 214)
(intern "*KEY_WORLD_55*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_55* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_55* 215)
(intern "*KEY_WORLD_56*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_56* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_56* 216)
(intern "*KEY_WORLD_57*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_57* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_57* 217)
(intern "*KEY_WORLD_58*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_58* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_58* 218)
(intern "*KEY_WORLD_59*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_59* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_59* 219)
(intern "*KEY_WORLD_60*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_60* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_60* 220)
(intern "*KEY_WORLD_61*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_61* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_61* 221)
(intern "*KEY_WORLD_62*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_62* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_62* 222)
(intern "*KEY_WORLD_63*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_63* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_63* 223)
(intern "*KEY_WORLD_64*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_64* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_64* 224)
(intern "*KEY_WORLD_65*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_65* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_65* 225)
(intern "*KEY_WORLD_66*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_66* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_66* 226)
(intern "*KEY_WORLD_67*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_67* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_67* 227)
(intern "*KEY_WORLD_68*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_68* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_68* 228)
(intern "*KEY_WORLD_69*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_69* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_69* 229)
(intern "*KEY_WORLD_70*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_70* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_70* 230)
(intern "*KEY_WORLD_71*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_71* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_71* 231)
(intern "*KEY_WORLD_72*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_72* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_72* 232)
(intern "*KEY_WORLD_73*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_73* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_73* 233)
(intern "*KEY_WORLD_74*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_74* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_74* 234)
(intern "*KEY_WORLD_75*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_75* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_75* 235)
(intern "*KEY_WORLD_76*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_76* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_76* 236)
(intern "*KEY_WORLD_77*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_77* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_77* 237)
(intern "*KEY_WORLD_78*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_78* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_78* 238)
(intern "*KEY_WORLD_79*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_79* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_79* 239)
(intern "*KEY_WORLD_80*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_80* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_80* 240)
(intern "*KEY_WORLD_81*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_81* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_81* 241)
(intern "*KEY_WORLD_82*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_82* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_82* 242)
(intern "*KEY_WORLD_83*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_83* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_83* 243)
(intern "*KEY_WORLD_84*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_84* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_84* 244)
(intern "*KEY_WORLD_85*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_85* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_85* 245)
(intern "*KEY_WORLD_86*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_86* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_86* 246)
(intern "*KEY_WORLD_87*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_87* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_87* 247)
(intern "*KEY_WORLD_88*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_88* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_88* 248)
(intern "*KEY_WORLD_89*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_89* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_89* 249)
(intern "*KEY_WORLD_90*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_90* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_90* 250)
(intern "*KEY_WORLD_91*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_91* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_91* 251)
(intern "*KEY_WORLD_92*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_92* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_92* 252)
(intern "*KEY_WORLD_93*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_93* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_93* 253)
(intern "*KEY_WORLD_94*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_94* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_94* 254)
(intern "*KEY_WORLD_95*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_WORLD_95* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_WORLD_95* 255)
(intern "*KEY_KP0*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_KP0* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_KP0* 256)
(intern "*KEY_KP1*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_KP1* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_KP1* 257)
(intern "*KEY_KP2*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_KP2* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_KP2* 258)
(intern "*KEY_KP3*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_KP3* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_KP3* 259)
(intern "*KEY_KP4*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_KP4* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_KP4* 260)
(intern "*KEY_KP5*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_KP5* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_KP5* 261)
(intern "*KEY_KP6*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_KP6* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_KP6* 262)
(intern "*KEY_KP7*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_KP7* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_KP7* 263)
(intern "*KEY_KP8*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_KP8* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_KP8* 264)
(intern "*KEY_KP9*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_KP9* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_KP9* 265)
(intern "*KEY_KP_PERIOD*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_KP_PERIOD* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_KP_PERIOD* 266)
(intern "*KEY_KP_DIVIDE*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_KP_DIVIDE* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_KP_DIVIDE* 267)
(intern "*KEY_KP_MULTIPLY*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_KP_MULTIPLY* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_KP_MULTIPLY* 268)
(intern "*KEY_KP_MINUS*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_KP_MINUS* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_KP_MINUS* 269)
(intern "*KEY_KP_PLUS*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_KP_PLUS* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_KP_PLUS* 270)
(intern "*KEY_KP_ENTER*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_KP_ENTER* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_KP_ENTER* 271)
(intern "*KEY_KP_EQUALS*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_KP_EQUALS* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_KP_EQUALS* 272)
(intern "*KEY_UP*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_UP* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_UP* 273)
(intern "*KEY_DOWN*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_DOWN* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_DOWN* 274)
(intern "*KEY_RIGHT*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_RIGHT* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_RIGHT* 275)
(intern "*KEY_LEFT*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_LEFT* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_LEFT* 276)
(intern "*KEY_INSERT*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_INSERT* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_INSERT* 277)
(intern "*KEY_HOME*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_HOME* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_HOME* 278)
(intern "*KEY_END*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_END* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_END* 279)
(intern "*KEY_PAGEUP*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_PAGEUP* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_PAGEUP* 280)
(intern "*KEY_PAGEDOWN*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_PAGEDOWN* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_PAGEDOWN* 281)
(intern "*KEY_F1*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_F1* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_F1* 282)
(intern "*KEY_F2*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_F2* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_F2* 283)
(intern "*KEY_F3*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_F3* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_F3* 284)
(intern "*KEY_F4*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_F4* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_F4* 285)
(intern "*KEY_F5*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_F5* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_F5* 286)
(intern "*KEY_F6*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_F6* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_F6* 287)
(intern "*KEY_F7*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_F7* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_F7* 288)
(intern "*KEY_F8*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_F8* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_F8* 289)
(intern "*KEY_F9*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_F9* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_F9* 290)
(intern "*KEY_F10*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_F10* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_F10* 291)
(intern "*KEY_F11*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_F11* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_F11* 292)
(intern "*KEY_F12*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_F12* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_F12* 293)
(intern "*KEY_F13*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_F13* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_F13* 294)
(intern "*KEY_F14*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_F14* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_F14* 295)
(intern "*KEY_F15*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_F15* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_F15* 296)
(intern "*KEY_NUMLOCK*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_NUMLOCK* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_NUMLOCK* 300)
(intern "*KEY_CAPSLOCK*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_CAPSLOCK* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_CAPSLOCK* 301)
(intern "*KEY_SCROLLOCK*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_SCROLLOCK* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_SCROLLOCK* 302)
(intern "*KEY_RSHIFT*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_RSHIFT* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_RSHIFT* 303)
(intern "*KEY_LSHIFT*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_LSHIFT* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_LSHIFT* 304)
(intern "*KEY_RCTRL*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_RCTRL* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_RCTRL* 305)
(intern "*KEY_LCTRL*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_LCTRL* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_LCTRL* 306)
(intern "*KEY_RALT*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_RALT* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_RALT* 307)
(intern "*KEY_LALT*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_LALT* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_LALT* 308)
(intern "*KEY_RMETA*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_RMETA* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_RMETA* 309)
(intern "*KEY_LMETA*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_LMETA* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_LMETA* 310)
(intern "*KEY_LSUPER*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_LSUPER* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_LSUPER* 311)
(intern "*KEY_RSUPER*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_RSUPER* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_RSUPER* 312)
(intern "*KEY_MODE*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_MODE* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_MODE* 313)
(intern "*KEY_COMPOSE*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_COMPOSE* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_COMPOSE* 314)
(intern "*KEY_HELP*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_HELP* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_HELP* 315)
(intern "*KEY_PRINT*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_PRINT* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_PRINT* 316)
(intern "*KEY_SYSREQ*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_SYSREQ* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_SYSREQ* 317)
(intern "*KEY_BREAK*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_BREAK* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_BREAK* 318)
(intern "*KEY_MENU*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_MENU* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_MENU* 319)
(intern "*KEY_POWER*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_POWER* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_POWER* 320)
(intern "*KEY_EURO*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_EURO* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_EURO* 321)
(intern "*KEY_UNDO*" (find-package "KEYBOARD::KEY"))
(shadow '*KEY_UNDO* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*KEY_UNDO* 322)
(intern "*MODIFIER_NONE*" (find-package "KEYBOARD::KEY"))
(shadow '*MODIFIER_NONE* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*MODIFIER_NONE* 0)
(intern "*MODIFIER_LSHIFT*" (find-package "KEYBOARD::KEY"))
(shadow '*MODIFIER_LSHIFT* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*MODIFIER_LSHIFT* 1)
(intern "*MODIFIER_RSHIFT*" (find-package "KEYBOARD::KEY"))
(shadow '*MODIFIER_RSHIFT* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*MODIFIER_RSHIFT* 2)
(intern "*MODIFIER_LCTRL*" (find-package "KEYBOARD::KEY"))
(shadow '*MODIFIER_LCTRL* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*MODIFIER_LCTRL* 64)
(intern "*MODIFIER_RCTRL*" (find-package "KEYBOARD::KEY"))
(shadow '*MODIFIER_RCTRL* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*MODIFIER_RCTRL* 128)
(intern "*MODIFIER_LALT*" (find-package "KEYBOARD::KEY"))
(shadow '*MODIFIER_LALT* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*MODIFIER_LALT* 256)
(intern "*MODIFIER_RALT*" (find-package "KEYBOARD::KEY"))
(shadow '*MODIFIER_RALT* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*MODIFIER_RALT* 512)
(intern "*MODIFIER_LMETA*" (find-package "KEYBOARD::KEY"))
(shadow '*MODIFIER_LMETA* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*MODIFIER_LMETA* 1024)
(intern "*MODIFIER_RMETA*" (find-package "KEYBOARD::KEY"))
(shadow '*MODIFIER_RMETA* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*MODIFIER_RMETA* 2048)
(intern "*MODIFIER_NUM*" (find-package "KEYBOARD::KEY"))
(shadow '*MODIFIER_NUM* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*MODIFIER_NUM* 4096)
(intern "*MODIFIER_CAPS*" (find-package "KEYBOARD::KEY"))
(shadow '*MODIFIER_CAPS* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*MODIFIER_CAPS* 8192)
(intern "*MODIFIER_MODE*" (find-package "KEYBOARD::KEY"))
(shadow '*MODIFIER_MODE* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*MODIFIER_MODE* 16384)
(intern "*MODIFIER_RESERVED*" (find-package "KEYBOARD::KEY"))
(shadow '*MODIFIER_RESERVED* (find-package "KEYBOARD::KEY"))
(defconstant keyboard::Key::*MODIFIER_RESERVED* 32768)
(defclass keyboard::Key
  :super ros::object
  :slots (_header _code _modifiers ))

(defmethod keyboard::Key
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:code __code) 0)
    ((:modifiers __modifiers) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _code (round __code))
   (setq _modifiers (round __modifiers))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:code
   (&optional __code)
   (if __code (setq _code __code)) _code)
  (:modifiers
   (&optional __modifiers)
   (if __modifiers (setq _modifiers __modifiers)) _modifiers)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint16 _code
    2
    ;; uint16 _modifiers
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint16 _code
       (write-word _code s)
     ;; uint16 _modifiers
       (write-word _modifiers s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint16 _code
     (setq _code (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _modifiers
     (setq _modifiers (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get keyboard::Key :md5sum-) "5d57616b5631968b8f1d31d23c83281f")
(setf (get keyboard::Key :datatype-) "keyboard/Key")
(setf (get keyboard::Key :definition-)
      "uint16 KEY_UNKNOWN=0
uint16 KEY_FIRST=0
uint16 KEY_BACKSPACE=8
uint16 KEY_TAB=9
uint16 KEY_CLEAR=12
uint16 KEY_RETURN=13
uint16 KEY_PAUSE=19
uint16 KEY_ESCAPE=27
uint16 KEY_SPACE=32
uint16 KEY_EXCLAIM=33
uint16 KEY_QUOTEDBL=34
uint16 KEY_HASH=35
uint16 KEY_DOLLAR=36
uint16 KEY_AMPERSAND=38
uint16 KEY_QUOTE=39
uint16 KEY_LEFTPAREN=40
uint16 KEY_RIGHTPAREN=41
uint16 KEY_ASTERISK=42
uint16 KEY_PLUS=43
uint16 KEY_COMMA=44
uint16 KEY_MINUS=45
uint16 KEY_PERIOD=46
uint16 KEY_SLASH=47
uint16 KEY_0=48
uint16 KEY_1=49
uint16 KEY_2=50
uint16 KEY_3=51
uint16 KEY_4=52
uint16 KEY_5=53
uint16 KEY_6=54
uint16 KEY_7=55
uint16 KEY_8=56
uint16 KEY_9=57
uint16 KEY_COLON=58
uint16 KEY_SEMICOLON=59
uint16 KEY_LESS=60
uint16 KEY_EQUALS=61
uint16 KEY_GREATER=62
uint16 KEY_QUESTION=63
uint16 KEY_AT=64
uint16 KEY_LEFTBRACKET=91
uint16 KEY_BACKSLASH=92
uint16 KEY_RIGHTBRACKET=93
uint16 KEY_CARET=94
uint16 KEY_UNDERSCORE=95
uint16 KEY_BACKQUOTE=96
uint16 KEY_a=97
uint16 KEY_b=98
uint16 KEY_c=99
uint16 KEY_d=100
uint16 KEY_e=101
uint16 KEY_f=102
uint16 KEY_g=103
uint16 KEY_h=104
uint16 KEY_i=105
uint16 KEY_j=106
uint16 KEY_k=107
uint16 KEY_l=108
uint16 KEY_m=109
uint16 KEY_n=110
uint16 KEY_o=111
uint16 KEY_p=112
uint16 KEY_q=113
uint16 KEY_r=114
uint16 KEY_s=115
uint16 KEY_t=116
uint16 KEY_u=117
uint16 KEY_v=118
uint16 KEY_w=119
uint16 KEY_x=120
uint16 KEY_y=121
uint16 KEY_z=122
uint16 KEY_DELETE=127
uint16 KEY_WORLD_0=160
uint16 KEY_WORLD_1=161
uint16 KEY_WORLD_2=162
uint16 KEY_WORLD_3=163
uint16 KEY_WORLD_4=164
uint16 KEY_WORLD_5=165
uint16 KEY_WORLD_6=166
uint16 KEY_WORLD_7=167
uint16 KEY_WORLD_8=168
uint16 KEY_WORLD_9=169
uint16 KEY_WORLD_10=170
uint16 KEY_WORLD_11=171
uint16 KEY_WORLD_12=172
uint16 KEY_WORLD_13=173
uint16 KEY_WORLD_14=174
uint16 KEY_WORLD_15=175
uint16 KEY_WORLD_16=176
uint16 KEY_WORLD_17=177
uint16 KEY_WORLD_18=178
uint16 KEY_WORLD_19=179
uint16 KEY_WORLD_20=180
uint16 KEY_WORLD_21=181
uint16 KEY_WORLD_22=182
uint16 KEY_WORLD_23=183
uint16 KEY_WORLD_24=184
uint16 KEY_WORLD_25=185
uint16 KEY_WORLD_26=186
uint16 KEY_WORLD_27=187
uint16 KEY_WORLD_28=188
uint16 KEY_WORLD_29=189
uint16 KEY_WORLD_30=190
uint16 KEY_WORLD_31=191
uint16 KEY_WORLD_32=192
uint16 KEY_WORLD_33=193
uint16 KEY_WORLD_34=194
uint16 KEY_WORLD_35=195
uint16 KEY_WORLD_36=196
uint16 KEY_WORLD_37=197
uint16 KEY_WORLD_38=198
uint16 KEY_WORLD_39=199
uint16 KEY_WORLD_40=200
uint16 KEY_WORLD_41=201
uint16 KEY_WORLD_42=202
uint16 KEY_WORLD_43=203
uint16 KEY_WORLD_44=204
uint16 KEY_WORLD_45=205
uint16 KEY_WORLD_46=206
uint16 KEY_WORLD_47=207
uint16 KEY_WORLD_48=208
uint16 KEY_WORLD_49=209
uint16 KEY_WORLD_50=210
uint16 KEY_WORLD_51=211
uint16 KEY_WORLD_52=212
uint16 KEY_WORLD_53=213
uint16 KEY_WORLD_54=214
uint16 KEY_WORLD_55=215
uint16 KEY_WORLD_56=216
uint16 KEY_WORLD_57=217
uint16 KEY_WORLD_58=218
uint16 KEY_WORLD_59=219
uint16 KEY_WORLD_60=220
uint16 KEY_WORLD_61=221
uint16 KEY_WORLD_62=222
uint16 KEY_WORLD_63=223
uint16 KEY_WORLD_64=224
uint16 KEY_WORLD_65=225
uint16 KEY_WORLD_66=226
uint16 KEY_WORLD_67=227
uint16 KEY_WORLD_68=228
uint16 KEY_WORLD_69=229
uint16 KEY_WORLD_70=230
uint16 KEY_WORLD_71=231
uint16 KEY_WORLD_72=232
uint16 KEY_WORLD_73=233
uint16 KEY_WORLD_74=234
uint16 KEY_WORLD_75=235
uint16 KEY_WORLD_76=236
uint16 KEY_WORLD_77=237
uint16 KEY_WORLD_78=238
uint16 KEY_WORLD_79=239
uint16 KEY_WORLD_80=240
uint16 KEY_WORLD_81=241
uint16 KEY_WORLD_82=242
uint16 KEY_WORLD_83=243
uint16 KEY_WORLD_84=244
uint16 KEY_WORLD_85=245
uint16 KEY_WORLD_86=246
uint16 KEY_WORLD_87=247
uint16 KEY_WORLD_88=248
uint16 KEY_WORLD_89=249
uint16 KEY_WORLD_90=250
uint16 KEY_WORLD_91=251
uint16 KEY_WORLD_92=252
uint16 KEY_WORLD_93=253
uint16 KEY_WORLD_94=254
uint16 KEY_WORLD_95=255
uint16 KEY_KP0=256
uint16 KEY_KP1=257
uint16 KEY_KP2=258
uint16 KEY_KP3=259
uint16 KEY_KP4=260
uint16 KEY_KP5=261
uint16 KEY_KP6=262
uint16 KEY_KP7=263
uint16 KEY_KP8=264
uint16 KEY_KP9=265
uint16 KEY_KP_PERIOD=266
uint16 KEY_KP_DIVIDE=267
uint16 KEY_KP_MULTIPLY=268
uint16 KEY_KP_MINUS=269
uint16 KEY_KP_PLUS=270
uint16 KEY_KP_ENTER=271
uint16 KEY_KP_EQUALS=272
uint16 KEY_UP=273
uint16 KEY_DOWN=274
uint16 KEY_RIGHT=275
uint16 KEY_LEFT=276
uint16 KEY_INSERT=277
uint16 KEY_HOME=278
uint16 KEY_END=279
uint16 KEY_PAGEUP=280
uint16 KEY_PAGEDOWN=281
uint16 KEY_F1=282
uint16 KEY_F2=283
uint16 KEY_F3=284
uint16 KEY_F4=285
uint16 KEY_F5=286
uint16 KEY_F6=287
uint16 KEY_F7=288
uint16 KEY_F8=289
uint16 KEY_F9=290
uint16 KEY_F10=291
uint16 KEY_F11=292
uint16 KEY_F12=293
uint16 KEY_F13=294
uint16 KEY_F14=295
uint16 KEY_F15=296
uint16 KEY_NUMLOCK=300
uint16 KEY_CAPSLOCK=301
uint16 KEY_SCROLLOCK=302
uint16 KEY_RSHIFT=303
uint16 KEY_LSHIFT=304
uint16 KEY_RCTRL=305
uint16 KEY_LCTRL=306
uint16 KEY_RALT=307
uint16 KEY_LALT=308
uint16 KEY_RMETA=309
uint16 KEY_LMETA=310
uint16 KEY_LSUPER=311
uint16 KEY_RSUPER=312
uint16 KEY_MODE=313
uint16 KEY_COMPOSE=314
uint16 KEY_HELP=315
uint16 KEY_PRINT=316
uint16 KEY_SYSREQ=317
uint16 KEY_BREAK=318
uint16 KEY_MENU=319
uint16 KEY_POWER=320
uint16 KEY_EURO=321
uint16 KEY_UNDO=322
uint16 MODIFIER_NONE=0
uint16 MODIFIER_LSHIFT=1
uint16 MODIFIER_RSHIFT=2
uint16 MODIFIER_LCTRL=64
uint16 MODIFIER_RCTRL=128
uint16 MODIFIER_LALT=256
uint16 MODIFIER_RALT=512
uint16 MODIFIER_LMETA=1024
uint16 MODIFIER_RMETA=2048
uint16 MODIFIER_NUM=4096
uint16 MODIFIER_CAPS=8192
uint16 MODIFIER_MODE=16384
uint16 MODIFIER_RESERVED=32768

Header header
uint16 code
uint16 modifiers

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

")



(provide :keyboard/Key "5d57616b5631968b8f1d31d23c83281f")


