import { ArmControlCenter } from "./arm/ArmControlCenter";

export interface AppProps {
  request?: typeof fetch;
}

export default function App({ request }: AppProps) {
  return (
    <main className="arm-dashboard-shell">
      <ArmControlCenter request={request} />
    </main>
  );
}
