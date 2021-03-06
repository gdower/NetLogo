// (C) Uri Wilensky. https://github.com/NetLogo/NetLogo

package org.nlogo.prim.etc

import org.nlogo.api.OutputDestination
import org.nlogo.nvm.{ Command, Context }

class _outputwrite extends Command {

  override def perform(context: Context) {
    workspace.outputObject(
      args(0).report(context), null, false, true,
      OutputDestination.OutputArea)
    context.ip = next
  }
}
