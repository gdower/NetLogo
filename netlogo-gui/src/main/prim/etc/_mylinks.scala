// (C) Uri Wilensky. https://github.com/NetLogo/NetLogo

package org.nlogo.prim.etc

import org.nlogo.agent.{ AgentSet, Turtle }
import org.nlogo.core.AgentKind
import org.nlogo.nvm.{ Context, Reporter }

class _mylinks(val breedName: String) extends Reporter {

  def this() = this(null)

  override def toString =
    super.toString + ":" + breedName

  override def report(context: Context): AnyRef = {
    val breed =
      if (breedName == null) world.links
      else world.getLinkBreed(breedName)
    AgentSet.fromArray(AgentKind.Link, world.linkManager.links(context.agent.asInstanceOf[Turtle], breed))
  }
}
