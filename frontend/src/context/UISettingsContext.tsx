import React, { createContext, useContext, useMemo, useState } from "react";

interface UISettingsContextType {
  viewReasoning: boolean;
  setViewReasoning: (value: boolean) => void;
  presentationMode: boolean;
  setPresentationMode: (value: boolean) => void;
}

const VIEW_REASONING_KEY = "ui:view_reasoning";
const PRESENTATION_MODE_KEY = "ui:presentation_mode";

const UISettingsContext = createContext<UISettingsContextType | undefined>(undefined);

export const UISettingsProvider: React.FC<{ children: React.ReactNode }> = ({
  children,
}) => {
  const [viewReasoning, setViewReasoningState] = useState<boolean>(() => {
    const stored = localStorage.getItem(VIEW_REASONING_KEY);
    return stored === "true";
  });
  const [presentationMode, setPresentationModeState] = useState<boolean>(() => {
    const stored = localStorage.getItem(PRESENTATION_MODE_KEY);
    return stored === "true";
  });

  const setViewReasoning = (value: boolean) => {
    setViewReasoningState(value);
    localStorage.setItem(VIEW_REASONING_KEY, String(value));
  };

  const setPresentationMode = (value: boolean) => {
    setPresentationModeState(value);
    localStorage.setItem(PRESENTATION_MODE_KEY, String(value));
  };

  const value = useMemo(
    () => ({ viewReasoning, setViewReasoning, presentationMode, setPresentationMode }),
    [viewReasoning, presentationMode]
  );

  return (
    <UISettingsContext.Provider value={value}>
      {children}
    </UISettingsContext.Provider>
  );
};

export const useUISettings = () => {
  const context = useContext(UISettingsContext);
  if (!context) {
    throw new Error("useUISettings must be used within a UISettingsProvider");
  }
  return context;
};
